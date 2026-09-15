/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FwFfControl.hpp"

#include <cstring>

using namespace time_literals;

ModuleBase::Descriptor FwFfControl::desc{task_spawn, custom_command, print_usage};

FwFfControl::FwFfControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_fw_feedforward_pub.advertise();
	_ff_sp_pub.advertise();
}

bool
FwFfControl::init()
{
	// run on a fixed interval: inputs arrive either via uORB (vehicle_ff_setpoint)
	// or MAVLink TUNNEL, and the integrator/timeout logic must advance regardless
	ScheduleOnInterval(10_ms); // 100 Hz

	return true;
}

float FwFfControl::getTrueAirspeed()
{
	_airspeed_validated_sub.update();

	// if no airspeed measurement is available our best guess is the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (PX4_ISFINITE(_airspeed_validated_sub.get().true_airspeed_m_s)
	    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s)) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().true_airspeed_m_s);

	} else {
		// fall back to the inertial speed estimate
		vehicle_local_position_s local_pos{};

		if (_local_pos_sub.copy(&local_pos) && local_pos.v_xy_valid) {
			airspeed = math::max(0.5f, sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy
							 + local_pos.vz * local_pos.vz));
		}
	}

	return airspeed;
}

void FwFfControl::mavlink_tunnel_poll()
{
	mavlink_tunnel_s tunnel{};

	while (_mavlink_tunnel_sub.update(&tunnel)) {
		if (tunnel.payload_type != kFfTunnelPayloadType || tunnel.payload_length < kFfTunnelPayloadLength) {
			continue;
		}

		vehicle_ff_setpoint_s ff_sp{};
		ff_sp.timestamp = hrt_absolute_time();
		memcpy(&ff_sp.normal_accel_ff, &tunnel.payload[0], sizeof(float));
		memcpy(&ff_sp.tangential_accel_ff, &tunnel.payload[4], sizeof(float));
		memcpy(&ff_sp.roll_rate_ff, &tunnel.payload[8], sizeof(float));
		_ff_sp_pub.publish(ff_sp);
	}
}

void FwFfControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// test injection: republish the injected setpoint at ~20 Hz while active
	const hrt_abstime now = hrt_absolute_time();

	if (now < _inject_until) {
		if (now - _last_inject_pub >= 50_ms) {
			_last_inject_pub = now;
			_inject_sp.timestamp = now;
			_ff_sp_pub.publish(_inject_sp);
		}
	}

	// only update parameters if they changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// forward feedforward setpoints arriving via MAVLink TUNNEL to the canonical topic
	mavlink_tunnel_poll();

	// Use the latest received setpoint: SubscriptionData keeps the last sample, and validity is
	// decided by the sample timestamp (not by "a new sample arrived in this cycle"), so the
	// feedforward keeps being applied until the input actually times out.
	_ff_sp_sub.update();
	const vehicle_ff_setpoint_s &ff_sp = _ff_sp_sub.get();

	const bool input_valid = (ff_sp.timestamp > 0)
				 && (hrt_elapsed_time(&ff_sp.timestamp) < kFfInputTimeout)
				 && PX4_ISFINITE(ff_sp.normal_accel_ff) && PX4_ISFINITE(ff_sp.tangential_accel_ff)
				 && PX4_ISFINITE(ff_sp.roll_rate_ff);

	fw_feedforward_s ff_out{};
	ff_out.timestamp = now;

	if (input_valid) {
		const float true_airspeed = getTrueAirspeed();

		ff_out.pitch_rate_ff = fw_ff::pitchRateFeedforward(_param_fw_nn_ff_gain.get(), true_airspeed,
				       ff_sp.normal_accel_ff, _param_fw_airspd_min.get(),
				       _param_fw_p_rmax_pos.get(), _param_fw_p_rmax_neg.get());
		ff_out.roll_rate_ff = fw_ff::rollRateFeedforward(_param_fw_p_ff_gain.get(), ff_sp.roll_rate_ff);
		// n_t_ff -> specific total energy rate, consumed by TECS as throttle feedforward
		ff_out.ste_rate_ff = fw_ff::totalEnergyRateFeedforward(_param_fw_nt_ff_gain.get(), true_airspeed,
				     ff_sp.tangential_accel_ff);
		ff_out.true_airspeed = true_airspeed;
		ff_out.valid = true;
	}

	_fw_feedforward_pub.publish(ff_out);
}

void FwFfControl::startInjection(float normal_accel_ff, float tangential_accel_ff, float roll_rate_ff, float duration_s)
{
	_inject_sp.normal_accel_ff = normal_accel_ff;
	_inject_sp.tangential_accel_ff = tangential_accel_ff;
	_inject_sp.roll_rate_ff = roll_rate_ff;
	_inject_until = hrt_absolute_time() + (hrt_abstime)(duration_s * 1e6f);
	ScheduleNow();
}

int FwFfControl::task_spawn(int argc, char *argv[])
{
	FwFfControl *instance = new FwFfControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FwFfControl::custom_command(int argc, char *argv[])
{
	if (argc >= 4 && !strcmp(argv[0], "inject")) {
		FwFfControl *instance = static_cast<FwFfControl *>(desc.object.load());

		if (!instance) {
			PX4_ERR("not running");
			return 1;
		}

		const float normal_accel_ff = strtof(argv[1], nullptr);
		const float tangential_accel_ff = strtof(argv[2], nullptr);
		const float roll_rate_ff = strtof(argv[3], nullptr);
		const float duration_s = (argc >= 5) ? strtof(argv[4], nullptr) : 5.f;

		instance->startInjection(normal_accel_ff, tangential_accel_ff, roll_rate_ff, duration_s);
		PX4_INFO("injecting n_n_ff=%.2f n_t_ff=%.2f p_ff=%.2f for %.1f s", (double)normal_accel_ff,
			 (double)tangential_accel_ff, (double)roll_rate_ff, (double)duration_s);
		return 0;
	}

	return print_usage("unknown command");
}

int FwFfControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_ff_control computes fixed-wing feedforward augmentation terms (u = u_ff + u_fb) from an external
time-parameterized trajectory planner publishing vehicle_ff_setpoint (normal/tangential load factor
and roll rate feedforward):

- normal load factor n_n_ff -> pitch rate feedforward (added to the attitude controller output)
- tangential load factor n_t_ff -> specific total energy rate feedforward (added to the TECS total
  energy rate demand, i.e. it drives the throttle through the existing energy rate -> throttle mapping)
- roll rate p_ff -> roll rate feedforward (added to the attitude controller output)

The computed increments are added on top of the existing TECS, attitude and rate feedback
controllers without modifying them.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_ff_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("inject", "Publish a constant vehicle_ff_setpoint for testing");
	PRINT_MODULE_USAGE_ARG("<n_n_ff> <n_t_ff> <p_ff> [<duration_s>]", "feedforward values and duration (default 5 s)", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_ff_control_main(int argc, char *argv[])
{
	return ModuleBase::main(FwFfControl::desc, argc, argv);
}
