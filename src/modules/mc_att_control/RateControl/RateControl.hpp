/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file RateControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>

#include <lib/mixer/mixer.h>
#include <uORB/topics/rate_ctrl_status.h>

class RateControl
{
public:
	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set update frequency and low-pass filter cutoff that is applied to the derivative term
	 * @param loop_rate [Hz] rate with which update function is called
	 * @param cutoff [Hz] cutoff frequency for the low-pass filter on the dervative term
	 * @param force flag to force an expensive update even if the cutoff didn't change
	 */
	void setDTermCutoff(const float loop_rate, const float cutoff, const bool force);

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param status message from mixer reporting about saturation
	 */
	void setSaturationStatus(const MultirotorMixer::saturation_status &status);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f rate, const matrix::Vector3f rate_sp, const float dt, const bool landed);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); }

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

	/**
	 * 	Get the
	 * 	@see z_k_Pr_R
	 * 	@return The z variable used by RCAC in the PID+FF controller
	 */
	const matrix::Vector3f get_RCAC_rate_z()
	{
		matrix::Vector3f RCAC_z{};

		for (int i = 0; i <= 2; i++) {
			RCAC_z(i) = z_k_rate(i); //z_k_AC_R(i,0);
		}

		return RCAC_z;
	}

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the PID+FF controller
	 */
	const matrix::Vector3f get_RCAC_rate_u()
	{
		matrix::Vector3f RCAC_u{};

		for (int i = 0; i <= 2; i++) {
			RCAC_u(i) = u_k_rate(i); //u_k_AC_R(i,0);
		}

		return RCAC_u;
	}

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the PID+FF controller
	 */
	const matrix::Matrix<float, 12,1> get_RCAC_rate_theta()
	{
		matrix::Matrix<float, 12,1> RCAC_theta{};

		for (int i = 0; i <= 3; i++) {
			// RCAC_theta(i,0) = theta_k_AC_R(i,0);
			RCAC_theta(i,0) = theta_k_rate_x(i,0);
			RCAC_theta(i+4,0) = theta_k_rate_y(i,0);
			RCAC_theta(i+8,0) = theta_k_rate_z(i,0);
		}

		return RCAC_theta;
	}

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC rate controller
	 */
	const int &get_RCAC_rate_ii() { return ii_AC_R; }

	/**
	 * 	Set the RCAC Rate switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_rate_switch(float switch_RCAC)
	{
		RCAC_Aw_ON = 1;
		if (switch_RCAC<0.0f) {
			RCAC_Aw_ON = 0;
		}
	}
private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_prev; ///< angular rates of previous update
	matrix::Vector3f _rate_prev_filtered; ///< low-pass filtered angular rates of previous update
	matrix::Vector3f _rate_int; ///< integral term of the rate controller
	math::LowPassFilter2pVector3f _lp_filters_d{0.f, 0.f}; ///< low-pass filters for D-term (roll, pitch & yaw)
	bool _mixer_saturation_positive[3] {};
	bool _mixer_saturation_negative[3] {};

	int ii_AC_R = 0;
  	bool RCAC_Aw_ON=1;
	// matrix::SquareMatrix<float, 12> P_AC_R;
	// matrix::Matrix<float, 3,12> phi_k_AC_R, phi_km1_AC_R;
	// matrix::Matrix<float, 12,1> theta_k_AC_R,theta_k_Ac_PID;
  	// matrix::Matrix<float, 3,1> z_k_AC_R, z_km1_AC_R,u_k_AC_R, u_km1_AC_R;

	// matrix::SquareMatrix<float, 3> Gamma_AC_R, I3, N1_Aw;

	matrix::SquareMatrix<float, 4> P_rate_x,P_rate_y,P_rate_z;
	matrix::Matrix<float, 1,4> phi_k_rate_x, phi_km1_rate_x;
	matrix::Matrix<float, 1,4> phi_k_rate_y, phi_km1_rate_y;
	matrix::Matrix<float, 1,4> phi_k_rate_z, phi_km1_rate_z;
	matrix::Matrix<float, 4,1> theta_k_rate_x, theta_k_rate_y, theta_k_rate_z;
  	matrix::Vector3f z_k_rate, z_km1_rate, u_k_rate, u_km1_rate;
	matrix::Vector3f N1_rate, Gamma_rate;
	matrix::Matrix<float, 1,1> dummy1,dummy2,dummy3;

	float alpha_P = 1.0f;
	float alpha_N = 1.0f;
};
