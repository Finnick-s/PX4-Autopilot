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

#include <gtest/gtest.h>
#include "FwFfControl.hpp"

TEST(FwFfControlTest, PitchRateFeedforwardLevelFlight)
{
	// n_n_ff = 1 (steady level flight) must produce zero pitch rate feedforward
	EXPECT_FLOAT_EQ(fw_ff::pitchRateFeedforward(1.f, 15.f, 1.f, 8.f, 45.f, 45.f), 0.f);

	// any gain with n_n_ff = 1 stays zero
	EXPECT_FLOAT_EQ(fw_ff::pitchRateFeedforward(5.f, 15.f, 1.f, 8.f, 45.f, 45.f), 0.f);
}

TEST(FwFfControlTest, PitchRateFeedforwardPullUp)
{
	// q_ff = K * g/V * (n_n_ff - 1)
	const float gain = 1.5f;
	const float airspeed = 12.f;
	const float n_n_ff = 1.5f;

	EXPECT_NEAR(fw_ff::pitchRateFeedforward(gain, airspeed, n_n_ff, 8.f, 90.f, 90.f),
		    gain * CONSTANTS_ONE_G / airspeed * (n_n_ff - 1.f), 1e-6f);

	// positive normal load factor must increase the pitch rate setpoint
	EXPECT_GT(fw_ff::pitchRateFeedforward(gain, airspeed, n_n_ff, 8.f, 90.f, 90.f), 0.f);

	// gain of zero disables the feedforward
	EXPECT_FLOAT_EQ(fw_ff::pitchRateFeedforward(0.f, airspeed, n_n_ff, 8.f, 90.f, 90.f), 0.f);
}

TEST(FwFfControlTest, PitchRateFeedforwardLowAirspeedFadeOut)
{
	// below the minimum airspeed the feedforward is faded out instead of blowing up as g/V
	EXPECT_FLOAT_EQ(fw_ff::pitchRateFeedforward(1.f, 0.f, 2.f, 8.f, 90.f, 90.f), 0.f);
	EXPECT_FLOAT_EQ(fw_ff::pitchRateFeedforward(1.f, 4.f, 2.f, 8.f, 90.f, 90.f), 0.f);

	// fully faded in above min_airspeed * (1 + kSpeedFadeRange)
	const float airspeed = 10.f;
	EXPECT_NEAR(fw_ff::pitchRateFeedforward(1.f, airspeed, 2.f, 8.f, 90.f, 90.f),
		    CONSTANTS_ONE_G / airspeed, 1e-6f);
}

TEST(FwFfControlTest, PitchRateFeedforwardRateLimit)
{
	// the increment is clamped to the configured pitch rate limits
	EXPECT_NEAR(fw_ff::pitchRateFeedforward(5.f, 10.f, 10.f, 8.f, 30.f, 15.f), math::radians(30.f), 1e-6f);
	EXPECT_NEAR(fw_ff::pitchRateFeedforward(5.f, 10.f, -10.f, 8.f, 30.f, 15.f), -math::radians(15.f), 1e-6f);
}

TEST(FwFfControlTest, RollRateFeedforward)
{
	EXPECT_FLOAT_EQ(fw_ff::rollRateFeedforward(1.f, 0.3f), 0.3f);
	EXPECT_FLOAT_EQ(fw_ff::rollRateFeedforward(0.f, 0.3f), 0.f);
	EXPECT_FLOAT_EQ(fw_ff::rollRateFeedforward(0.5f, -0.4f), -0.2f);
}

TEST(FwFfControlTest, TotalEnergyRateFeedforward)
{
	// E_dot_ff = gain * n_t_ff * V_TAS * g, added to the TECS total energy rate demand
	const float gain = 2.f;
	const float true_airspeed = 18.f;
	const float n_t_ff = 0.25f;

	EXPECT_NEAR(fw_ff::totalEnergyRateFeedforward(gain, true_airspeed, n_t_ff),
		    gain * n_t_ff * true_airspeed * CONSTANTS_ONE_G, 1e-6f);

	// zero tangential load factor demands no additional energy rate
	EXPECT_FLOAT_EQ(fw_ff::totalEnergyRateFeedforward(gain, true_airspeed, 0.f), 0.f);

	// gain of zero disables the feedforward
	EXPECT_FLOAT_EQ(fw_ff::totalEnergyRateFeedforward(0.f, true_airspeed, n_t_ff), 0.f);
}
