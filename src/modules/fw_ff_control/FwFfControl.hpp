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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fw_feedforward.h>
#include <uORB/topics/mavlink_tunnel.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_ff_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

namespace fw_ff
{

/** Relative airspeed band above the minimum airspeed over which the feedforward fades in. */
constexpr float kSpeedFadeRange{0.2f};

/**
 * Normal load factor to pitch rate feedforward using the short-period approximation
 * n_n - 1 ~ V/g * q  =>  q_ff = gain * g/V * (n_n_ff - 1)
 *
 * The feedforward is faded out below the minimum airspeed to avoid the g/V term growing
 * without bound at low speed and clamped to the configured pitch rate limits.
 */
inline float pitchRateFeedforward(float gain, float true_airspeed, float normal_accel_ff, float min_airspeed,
				  float p_rmax_pos_deg, float p_rmax_neg_deg)
{
	const float speed_floor = math::max(min_airspeed, 0.1f);
	const float fade = math::constrain((true_airspeed - speed_floor) / (kSpeedFadeRange * speed_floor), 0.f, 1.f);
	const float q_ff = fade * gain * CONSTANTS_ONE_G / math::max(true_airspeed, speed_floor) * (normal_accel_ff - 1.f);

	return math::constrain(q_ff, -math::radians(p_rmax_neg_deg), math::radians(p_rmax_pos_deg));
}

inline float rollRateFeedforward(float gain, float roll_rate_ff)
{
	return gain * roll_rate_ff;
}

/**
 * Tangential load factor to specific total energy rate feedforward.
 * Tangential load factor: n_t = (T - D)/(m*g) = V_dot/g + sin(gamma)
 * Specific total energy rate: E_dot = V*V_dot + g*h_dot = V*g*n_t
 *
 * The result [m²/s³] is added to the TECS total energy rate demand, which drives the throttle
 * through the existing energy rate -> throttle feedforward mapping.
 */
inline float totalEnergyRateFeedforward(float gain, float true_airspeed, float tangential_accel_ff)
{
	return gain * tangential_accel_ff * true_airspeed * CONSTANTS_ONE_G;
}

} // namespace fw_ff

class FwFfControl final : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	FwFfControl();
	~FwFfControl() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	static constexpr hrt_abstime kFfInputTimeout{500_ms};

	// MAVLink transport: outer-loop planners on a companion computer send the
	// feedforward setpoint as a MAVLink TUNNEL message (payload 3 x float32 LE:
	// normal_accel_ff, tangential_accel_ff, roll_rate_ff). mavlink_receiver
	// forwards unknown payload types to the mavlink_tunnel uORB topic, from where
	// it is republished here as vehicle_ff_setpoint (the canonical input topic).
	static constexpr uint16_t kFfTunnelPayloadType{0xFFF1}; // > 32767: local experiment
	static constexpr uint8_t kFfTunnelPayloadLength{12};    // 3 x float32

	uORB::SubscriptionData<vehicle_ff_setpoint_s> _ff_sp_sub{ORB_ID(vehicle_ff_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _mavlink_tunnel_sub{ORB_ID(mavlink_tunnel)};

	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<fw_feedforward_s> _fw_feedforward_pub{ORB_ID(fw_feedforward)};
	uORB::Publication<vehicle_ff_setpoint_s> _ff_sp_pub{ORB_ID(vehicle_ff_setpoint)}; // only used by the inject test command

	// inject test command state
	hrt_abstime _inject_until{0};
	hrt_abstime _last_inject_pub{0};
	vehicle_ff_setpoint_s _inject_sp{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_NN_FF_GAIN>) _param_fw_nn_ff_gain,
		(ParamFloat<px4::params::FW_NT_FF_GAIN>) _param_fw_nt_ff_gain,
		(ParamFloat<px4::params::FW_P_FF_GAIN>) _param_fw_p_ff_gain,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos
	)

	float getTrueAirspeed();
	void startInjection(float normal_accel_ff, float tangential_accel_ff, float roll_rate_ff, float duration_s);
	void mavlink_tunnel_poll();
};
