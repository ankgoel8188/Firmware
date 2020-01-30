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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

// #include <fstream>
// #include <iostream>

// using namespace std;
using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain)
{
	_proportional_gain = proportional_gain;

	// prepare yaw weight from the ratio between roll/pitch and yaw gains
	const float roll_pitch_gain = (proportional_gain(0) + proportional_gain(1)) / 2.f;
	_yaw_w = math::constrain(proportional_gain(2) / roll_pitch_gain, 0.f, 1.f);

	_proportional_gain(2) = roll_pitch_gain;
}

matrix::Vector3f AttitudeControl::update(matrix::Quatf q, matrix::Quatf qd, const float yawspeed_feedforward,const bool landed)
{
	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	q.normalize();
	qd.normalize();

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= q;
	}

	// mix full and reduced desired attitude
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix *= math::signNoZero(q_mix(0));
	// catch numerical problems with the domain of acosf and asinf
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	// calculate angular rates setpoint
	matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	if (landed)
	{
		ii_Pq_R = 0;
	}

	if ((RCAC_Aq_ON) && (!landed))
	// if (!_vehicle_land_detected.maybe_landed && !_vehicle_land_detected.landed && RCAC_Pq_ON)
	{
		// cout << "Attitude controller" << "\n";
		//vel_sp_position = 1.0f*(_pos_sp - _pos).emult(Vector3f(1.0f, 1.0f, 1.0f));
		ii_Pq_R = ii_Pq_R + 1;
		if (ii_Pq_R == 1)
		{
			P_Pq_R = eye<float, 3>() * 0.010 * 100.0f*alpha_P * 0.01;
			N1_Pq = eye<float, 3>() * (1.0f) * alpha_N;
			I3 = eye<float, 3>();
			phi_k_Pq_R.setZero();
			phi_km1_Pq_R.setZero();
			theta_k_Pq_R.setZero();
			z_k_Pq_R.setZero();
			z_km1_Pq_R.setZero();
			u_k_Pq_R.setZero();
			u_km1_Pq_R.setZero();
			Gamma_Pq_R.setZero();

			theta_k_Pq_R = 0.0f*_proportional_gain;
			theta_k_Pq_PID(0,0) = _proportional_gain(0);
			theta_k_Pq_PID(1,0) = _proportional_gain(1);
			theta_k_Pq_PID(2,0) = _proportional_gain(2);

		}

		phi_k_Pq_R(0, 0) = eq(0);
		phi_k_Pq_R(1, 1) = eq(1);
		phi_k_Pq_R(2, 2) = eq(2);

		z_k_Pq_R = eq;

		Gamma_Pq_R 	= phi_km1_Pq_R * P_Pq_R * phi_km1_Pq_R.T() + I3;
		Gamma_Pq_R 	= Gamma_Pq_R.I();
		P_Pq_R 		= P_Pq_R - (P_Pq_R * phi_km1_Pq_R.T()) * Gamma_Pq_R * (phi_km1_Pq_R * P_Pq_R);
		//theta_k_Pq_R 	= theta_k_Pq_R + (P_Pq_R * phi_km1_Pq_R.T()) *
		//		 (z_k_Pq_R + (-1.0f)*(phi_km1_Pq_R * theta_k_Pq_R - u_km1_Pq_R) * (-1.0f));
		theta_k_Pq_R 	= theta_k_Pq_R + (P_Pq_R * phi_km1_Pq_R.T()) * N1_Pq *
				 (z_k_Pq_R + N1_Pq*(phi_km1_Pq_R * theta_k_Pq_R - u_km1_Pq_R) );

		u_k_Pq_R 	= phi_k_Pq_R * (1.0f*theta_k_Pq_R+1.0f*theta_k_Pq_PID);
		u_km1_Pq_R 	= u_k_Pq_R;
		phi_km1_Pq_R 	= phi_k_Pq_R;

		rate_setpoint = u_k_Pq_R;

		/*if (1) //
		{
			//cout << "Writing RCAC_data.txt" << "\t" << dt << "\n";
			ofstream RCAC_A_q("RCAC_A_q.txt", std::fstream::in | std::fstream::out | std::fstream::app);
			if (RCAC_A_q.is_open())
			{
				RCAC_A_q << ii_Pq_R << "\t"
					<< z_k_Pq_R(0,0) << "\t"
					<< z_k_Pq_R(1,0) << "\t"
					<< z_k_Pq_R(2,0) << "\t"
					<< theta_k_Pq_R(0,0) << "\t"
					<< theta_k_Pq_R(1,0) << "\t"
					<< theta_k_Pq_R(2,0) << "\t"
					<< u_k_Pq_R(0,0) << "\t"
					<< u_k_Pq_R(1,0) << "\t"
					<< u_k_Pq_R(2,0) << "\t"
					//
					<< _proportional_gain(0) << "\t"
					<< _proportional_gain(1) << "\t"
					<< _proportional_gain(2) << "\t"
					<< "\n";
				RCAC_A_q.close();
			}
		}*/


	}



	// Feed forward the yaw setpoint rate.
	// yaw_sp_move_rate is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yaw_sp_move_rate).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	rate_setpoint += q.inversed().dcm_z() * yawspeed_feedforward;

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
