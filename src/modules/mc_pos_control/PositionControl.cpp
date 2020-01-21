/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "Utility/ControlMath.hpp"
#include <px4_defines.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace matrix;
using namespace std;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::_setCtrlFlag(bool value)
{
	for (int i = 0; i <= 2; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = value;
	}
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// by default we use the entire position-velocity control-loop pipeline (flag only for logging purpose)
	_setCtrlFlag(true);

	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acc_x, setpoint.acc_y, setpoint.acc_z);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping();

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))
			   && PX4_ISFINITE(_thr_sp(2));

	return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {

		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > _param_mpc_thr_max.get()) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_thr_max.get();

		} else if (thr_mag < _param_mpc_manthr_min.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_manthr_min.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} else {
		_positionController();
		_velocityController(dt);
	}
}

bool PositionControl::_interfaceMapping()
{
	// if nothing is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valid position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// States and setpoints which are integrals of the reference setpoint are set to 0.
	// For instance: reference is velocity-setpoint -> position and position-setpoint = 0
	//               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = NAN;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_ctrl_pos[i] = false; // position control-loop is not used

			// thrust setpoint is not supported in velocity control
			_thr_sp(i) = NAN;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_vel_sp(i) = _vel(i) = 0.0f;
			_ctrl_pos[i] = _ctrl_vel[i] = false; // position/velocity control loop is not used

			// Reset the Integral term.
			_thr_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(_param_mpc_thr_min.get() + (_param_mpc_thr_hover.get() - _param_mpc_thr_min.get()) * 0.7f);
		// position and velocity control-loop is currently unused (flag only for logging purpose)
		_setCtrlFlag(false);
	}

	return !(failsafe);
}

void PositionControl::_positionController()
{
	// P-position controller
        // const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
        //				 _param_mpc_z_p.get()));
        Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
                                         _param_mpc_z_p.get()));

	if (RCAC_Pr_ON)
	{
		// Adding a stupid comment to test git transfer 
		//vel_sp_position = 1.0f*(_pos_sp - _pos).emult(Vector3f(1.0f, 1.0f, 1.0f));
		ii_Pr_R = ii_Pr_R + 1;
		if (ii_Pr_R == 1)
		{
			P_Pr_R = eye<float, 3>() * 0.010 * alpha_P;
			N1_Pr = eye<float, 3>() * (1.0f)*alpha_N;
			I3 = eye<float, 3>();
			phi_k_Pr_R.setZero();
			phi_km1_Pr_R.setZero();
			theta_k_Pr_R.setZero();
			z_k_Pr_R.setZero();
			z_km1_Pr_R.setZero();
			u_k_Pr_R.setZero();
			u_km1_Pr_R.setZero();
			Gamma_Pr_R.setZero();

			theta_k_Pr_R = 0.0f*Vector3f(_param_mpc_xy_p.get(),
						_param_mpc_xy_p.get(),
						_param_mpc_z_p.get());

		}

		phi_k_Pr_R(0, 0) = _pos_sp(0) - _pos(0);
		phi_k_Pr_R(1, 1) = _pos_sp(1) - _pos(1);
		phi_k_Pr_R(2, 2) = _pos_sp(2) - _pos(2);

		z_k_Pr_R = (_pos_sp - _pos);

		Gamma_Pr_R 	= phi_km1_Pr_R * P_Pr_R * phi_km1_Pr_R.T() + I3;
		Gamma_Pr_R 	= Gamma_Pr_R.I();
		

		// if (abs(z_k_Pr_R(2,0))<0.5f)
		{
		P_Pr_R 		= P_Pr_R - (P_Pr_R * phi_km1_Pr_R.T()) * Gamma_Pr_R * (phi_km1_Pr_R * P_Pr_R);
		//theta_k_Pr_R 	= theta_k_Pr_R + (P_Pr_R * phi_km1_Pr_R.T()) *
		//		 (z_k_Pr_R + (-1.0f)*(phi_km1_Pr_R * theta_k_Pr_R - u_km1_Pr_R) * (-1.0f));
		theta_k_Pr_R 	= theta_k_Pr_R + (P_Pr_R * phi_km1_Pr_R.T()) * N1_Pr *
				 (z_k_Pr_R + N1_Pr*(phi_km1_Pr_R * theta_k_Pr_R - u_km1_Pr_R) );
		u_km1_Pr_R 		= phi_k_Pr_R * theta_k_Pr_R;
		phi_km1_Pr_R 	= phi_k_Pr_R;
		}
		u_k_Pr_R 	= phi_k_Pr_R * (theta_k_Pr_R+1.0f*Vector3f(_param_mpc_xy_p.get(),
																_param_mpc_xy_p.get(),
																_param_mpc_z_p.get()));
		u_km1_Pr_R 	= u_k_Pr_R;
		phi_km1_Pr_R 	= phi_k_Pr_R;

		// cout 	<< theta_k_Pr_R(0,0) << "\t"
		// 	<< theta_k_Pr_R(1,0) << "\t"
		// 	<< theta_k_Pr_R(2,0) << "\n";


		// cout 	<< ii_Pr_R << "\t"
		// 		// << theta_k_Pr_R(0,0) << "\t"
		// 		// << theta_k_Pr_R(1,0) << "\t"
		// 		<< theta_k_Pr_R(2,0) << "\t"
		// 		// << P_Pr_R(0,1)+P_Pr_R(0,2)+
		// 		//    P_Pr_R(1,0)+P_Pr_R(1,2)+
		// 		//    P_Pr_R(2,0)+P_Pr_R(2,1) << "\t"
		// 		<< P_Pr_R(2,2) << "\t" 
		// 		<< "\n";
		vel_sp_position = u_k_Pr_R;

		if (1) //
		{
			//cout << "Writing RCAC_data.txt" << "\t" << dt << "\n";
			ofstream RCAC_P_r("RCAC_P_r.txt", std::fstream::in | std::fstream::out | std::fstream::app);
			if (RCAC_P_r.is_open())
			{
				RCAC_P_r << ii_Pr_R << "\t"
						<< z_k_Pr_R(0,0) << "\t"
						<< z_k_Pr_R(1,0) << "\t"
						<< z_k_Pr_R(2,0) << "\t"
						<< _pos_sp(0) << "\t"
						<< _pos_sp(1) << "\t"
						<< _pos_sp(2) << "\t"
						<< _pos(0) << "\t"
						<< _pos(1) << "\t"
						<< _pos(2) << "\t"
						<< theta_k_Pr_R(0,0) << "\t"
						<< theta_k_Pr_R(1,0) << "\t"
						<< theta_k_Pr_R(2,0) << "\t"
						<< u_k_Pr_R(0,0) << "\t"
						<< u_k_Pr_R(1,0) << "\t"
						<< u_k_Pr_R(2,0) << "\t"
						//
						<< _param_mpc_xy_p.get() << "\t"
						<< _param_mpc_xy_p.get() << "\t"
						<< _param_mpc_z_p.get() << "\t"
						<< "\n";
				RCAC_P_r.close();
			}
		}
	}
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityController(const float &dt)
{
	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;
	// float thrust_desired_D = 0.0;

	// Consider thrust in D-direction.
	float thrust_desired_D = _param_mpc_z_vel_p.get() * vel_err(2) +
							 _param_mpc_z_vel_d.get() * _vel_dot(2) + 
							 _thr_int(2) - 
							 _param_mpc_thr_hover.get();

	if (!RCAC_Pv_ON)
	{
		// float thrust_desired_D = _param_mpc_z_vel_p.get() * vel_err(2) + _param_mpc_z_vel_d.get() * _vel_dot(2) + _thr_int(2) - _param_mpc_thr_hover.get();
		ii_R = ii_R + 1;
		/*int Gain = 1;
		if ((ii_R>1500) && (0))
		{
			Gain = -1;
			cout << "Flipped \n";
		}*/
		thrust_desired_D = 1*(_param_mpc_z_vel_p.get() * vel_err(2) +
				1*_param_mpc_z_vel_d.get() * _vel_dot(2) +
				1*_thr_int(2) -
				1*_param_mpc_thr_hover.get());
		// cout << _vel_sp(2) << "\t" << _vel(2) << "\t" << thrust_desired_D << "\n";		
		// if (ii_R%100==0)
		// if (ii_R<100)
		// Check that the integrator state is being reset after landing
		// It is!! So stop worrying about it. 
		// {
		// cout 	<< ii_R << "\t"
		// 	<< vel_err(0) << "\t"
		// 	<< vel_err(1) << "\t"
		// 	<< vel_err(2) << "\t \t"
		// 	<< _thr_int(0) << "\t" 
		// 	<< _thr_int(1) << "\t"
		// 	<< _thr_int(2) << "\t \t"
		// 	<< _vel_dot(0) << "\t"
		// 	<< _vel_dot(1) << "\t"
		// 	<< _vel_dot(2) << "\t"
		// 	<< "\n";
		// }
		if (0) //
			{
				ofstream PX4_PID_Data("PX4_PID_Data.txt", std::fstream::in | std::fstream::out | std::fstream::app);
				if (PX4_PID_Data.is_open())
				{

					PX4_PID_Data << 1 << "\t"
							  << dt << "\t"
							  << vel_err(2) << "\t"
							  << vel_err(2) << "\t"
							  << 0 << "\t"
							  << _thr_int(2) << "\t"
							  << _param_mpc_z_vel_p.get() << "\t"
							  << _param_mpc_z_vel_d.get() << "\t"
							  << _param_mpc_z_vel_i.get() << "\t"
							  << thrust_desired_D << "\t"
							  << _vel_sp(2) << "\t"
							  << _vel(2) << "\t"
							  << "\n";
					PX4_PID_Data.close();
				}
			}
	}
	else
	{
		if (dt > 0.01f)
		{
			// My shit here
			ii_R = ii_R + 1;
			/*cout << "Printing " << ii_R << " " << vel_err(0) << " "
						<< vel_err(1) << " "
						<< vel_err(2) << " "
						//<< _vel_sp(0) << " "
						//<< _vel_sp(1) << " "
						//<< _vel_sp(2) << " "
						<< _param_mpc_xy_vel_p.get() << " "
						<< _param_mpc_xy_vel_d.get() << " "
						<< _param_mpc_z_vel_i.get() << " "
	                                        << thrust_desired_D << " "
	  					<< dt  << " \n";   //Ankit Aug13
	*/
			// if (ii_R == 1)
			// {

			// 	P_x_R = eye<float, 3>() * 0.010;
			// 	P_y_R = eye<float, 3>() * 0.010;
			// 	P_z_R = eye<float, 3>() * 0.010;

			// 	testMat = eye<float,2>()*10;

			// 	cout << testMat(0,0) << "\n";

			// 	testMat = testMat.I();
			// 	cout << testMat(0,0) << "\n";

			// 	phi_k_x_R.setZero();
			// 	phi_k_y_R.setZero();
			// 	phi_k_z_R.setZero();
			// 	phi_km1_x_R.setZero();
			// 	phi_km1_y_R.setZero();
			// 	phi_km1_z_R.setZero();

			// 	theta_k_x_R.setZero();
			// 	theta_k_y_R.setZero();
			// 	theta_k_z_R.setZero();

			// 	z_k_x_R.setZero();
			// 	z_k_y_R.setZero();
			// 	z_k_z_R.setZero();

			// 	z_km1_x_R.setZero();
			// 	z_km1_y_R.setZero();
			// 	z_km1_z_R.setZero();

			// 	u_k_x_R.setZero();
			// 	u_k_y_R.setZero();
			// 	u_k_z_R.setZero();

			// 	u_km1_x_R.setZero();
			// 	u_km1_y_R.setZero();
			// 	u_km1_z_R.setZero();

			// 	Gamma_x_R.setZero();
			// 	Gamma_y_R.setZero();
			// 	Gamma_z_R.setZero();
			// }
			// //thrust_desired_D = -.13;

			// // phi_k_x_R(0, 0) = vel_err(2);
			// // phi_k_x_R(0, 1) = _vel_dot(2) * 0;
			// // phi_k_x_R(0, 2) = phi_k_x_R(0, 2) + vel_err(2) * dt;
			// // phi_k_x_R(0, 2) = _thr_int(2);


			// phi_k_x_R(0, 0) = vel_err(0);
			// phi_k_x_R(0, 1) = _vel_dot(0) * 0;
			// phi_k_x_R(0, 2) = _thr_int(0);

			// phi_k_y_R(0, 0) = vel_err(1);
			// phi_k_y_R(0, 1) = _vel_dot(1) * 0;
			// phi_k_y_R(0, 2) = _thr_int(1);

			// phi_k_z_R(0, 0) = vel_err(2);
			// phi_k_z_R(0, 1) = _vel_dot(2) * 0;
			// phi_k_z_R(0, 2) = _thr_int(2);



			// z_k_x_R(0, 0) = vel_err(0);
			// z_k_y_R(0, 0) = vel_err(1);
			// z_k_z_R(0, 0) = vel_err(2);

			// Gamma_x_R 	= phi_km1_x_R * P_x_R * phi_km1_x_R.T() + 1;
			// P_x_R 		= P_x_R - (P_x_R * phi_km1_x_R.T()) * (phi_km1_x_R * P_x_R) / Gamma_x_R(0, 0);
			// theta_k_x_R 	= theta_k_x_R + (P_x_R * phi_km1_x_R.T()) *
			// 		 (z_k_x_R + (-1.0f)*(phi_km1_x_R * theta_k_x_R - u_km1_x_R) * (-1.0f));
			// u_k_x_R 	= phi_k_x_R * theta_k_x_R;
			// u_km1_x_R 	= u_k_x_R;
			// phi_km1_x_R 	= phi_k_x_R;


			// Gamma_y_R 	= phi_km1_y_R * P_y_R * phi_km1_y_R.T() + 1;
			// P_y_R 		= P_y_R - (P_y_R * phi_km1_y_R.T()) * (phi_km1_y_R * P_y_R) / Gamma_y_R(0, 0);
			// theta_k_y_R 	= theta_k_y_R + (P_y_R * phi_km1_y_R.T()) *
			// 		 (z_k_y_R + (-1.0f)*(phi_km1_y_R * theta_k_y_R - u_km1_y_R) * (-1.0f));
			// u_k_y_R 	= phi_k_y_R * theta_k_y_R;
			// u_km1_y_R 	= u_k_y_R;
			// phi_km1_y_R 	= phi_k_y_R;

			// Gamma_z_R 	= phi_km1_z_R * P_z_R * phi_km1_z_R.T() + 1;
			// P_z_R 		= P_z_R - (P_z_R * phi_km1_z_R.T()) * (phi_km1_z_R * P_z_R) / Gamma_z_R(0, 0);
			// theta_k_z_R 	= theta_k_z_R + (P_z_R * phi_km1_z_R.T()) *
			// 		 (z_k_z_R + (-1.0f)*(phi_km1_z_R * theta_k_z_R - u_km1_z_R) * (-1.0f));
			// u_k_z_R 	= phi_k_z_R * theta_k_z_R;
			// u_km1_z_R 	= u_k_z_R;
			// phi_km1_z_R 	= phi_k_z_R;


			// thrust_desired_D = u_k_z_R(0, 0);

			/*cout << ii_R << "\t" << u_k_x_R(0, 0)
					<< "\t" << u_k_y_R(0, 0)
					<< "\t" << u_k_z_R(0, 0)
					<< "\t" << z_k_x_R(0, 0)
					<< "\t" << z_k_y_R(0, 0)
					<< "\t" << z_k_z_R(0, 0)
					<< "\n"; */

			ii_Pv_R = ii_Pv_R + 1;
			if (ii_Pv_R == 1)
			{
				P_Pv_R = eye<float, 9>() * 0.010 * alpha_P *0.1;
				N1_Pv = eye<float, 3>() * (1.0f) * alpha_N;
				I3 = eye<float, 3>();
				phi_k_Pv_R.setZero();
				phi_km1_Pv_R.setZero();
				theta_k_Pv_R.setZero();
				z_k_Pv_R.setZero();
				z_km1_Pv_R.setZero();
				u_k_Pv_R.setZero();
				u_km1_Pv_R.setZero();
				Gamma_Pv_R.setZero();

				theta_k_Pv_PID(0,0) = _param_mpc_xy_vel_p.get();
				theta_k_Pv_PID(1,0) = 1; //_param_mpc_xy_vel_i.get();
				theta_k_Pv_PID(2,0) = _param_mpc_xy_vel_d.get();
				theta_k_Pv_PID(3,0) = _param_mpc_xy_vel_p.get();
				theta_k_Pv_PID(4,0) = 1; //_param_mpc_xy_vel_i.get();
				theta_k_Pv_PID(5,0) = _param_mpc_xy_vel_d.get();
				theta_k_Pv_PID(6,0) = _param_mpc_z_vel_p.get();
				theta_k_Pv_PID(7,0) = 1; //_param_mpc_z_vel_i.get();
				theta_k_Pv_PID(8,0) = _param_mpc_z_vel_d.get();
				// theta_k_Pv_R = 0.0f*Vector3f(_param_mpc_z_vel_p.get(),
				// 			_param_mpc_z_vel_i.get(),
				// 			_param_mpc_z_vel_d.get());

			}



			phi_k_Pv_R(0, 0) = vel_err(0);
			phi_k_Pv_R(0, 1) = _thr_int(0) ; //_param_mpc_xy_vel_i.get();
			phi_k_Pv_R(0, 2) = _vel_dot(0) * 0;

			phi_k_Pv_R(1, 3) = vel_err(1);
			phi_k_Pv_R(1, 4) = _thr_int(1) ; //_param_mpc_xy_vel_i.get();
			phi_k_Pv_R(1, 5) = _vel_dot(1) * 0;

			phi_k_Pv_R(2, 6) = vel_err(2);
			phi_k_Pv_R(2, 7) = _thr_int(2) ; //_param_mpc_z_vel_i.get();
			phi_k_Pv_R(2, 8) = _vel_dot(2) * 0;

			Pv_intg(0) = Pv_intg(0) + _param_mpc_xy_vel_i.get() * vel_err(0) * dt;
			Pv_intg(1) = Pv_intg(1) + _param_mpc_xy_vel_i.get() * vel_err(1) * dt;
			Pv_intg(2) = Pv_intg(2) + _param_mpc_z_vel_i.get() * vel_err(2) * dt;

			// phi_k_Pv_R(0, 1) = Pv_intg(0);
			// phi_k_Pv_R(1, 4) = Pv_intg(1);
			// phi_k_Pv_R(2, 7) = Pv_intg(2);

			z_k_Pv_R 	= vel_err;

			Gamma_Pv_R 	= phi_km1_Pv_R * P_Pv_R * phi_km1_Pv_R.T() + I3;
			Gamma_Pv_R 	= Gamma_Pv_R.I();

			// if ((z_k_Pv_R(2,0))<0.5f)
			{
				P_Pv_R 		= P_Pv_R - (P_Pv_R * phi_km1_Pv_R.T()) * Gamma_Pv_R * (phi_km1_Pv_R * P_Pv_R);
				//theta_k_Pv_R 	= theta_k_Pv_R + (P_Pv_R * phi_km1_Pv_R.T()) *
				//		 (z_k_Pv_R + (-1.0f)*(phi_km1_Pv_R * theta_k_Pv_R - u_km1_Pv_R) * (-1.0f));
				theta_k_Pv_R 	= theta_k_Pv_R + (P_Pv_R * phi_km1_Pv_R.T()) * N1_Pv *
						(z_k_Pv_R + N1_Pv*(phi_km1_Pv_R * theta_k_Pv_R - u_km1_Pv_R) );
				// u_km1_Pv_R 		= phi_k_Pv_R * theta_k_Pv_R;;
				// phi_km1_Pv_R 	= phi_k_Pv_R;
			}
			u_k_Pv_R 	= phi_k_Pv_R * (1.0f*theta_k_Pv_R+0.0f*theta_k_Pv_PID);
			u_km1_Pv_R 	= u_k_Pv_R;
			phi_km1_Pv_R 	= phi_k_Pv_R;



			
			// cout 	<< u_k_Pv_R(0,0) << "\t"
			// 		<< u_k_Pv_R(1,0) << "\t"
			// 		<< u_k_Pv_R(2,0) << "\n";

			


			// cout 	<< _thr_int(0) << "\t"
			// 		<< _thr_int(1) << "\t"
			// 		<< _thr_int(2) << "\t"
			// 		<< Pv_intg(0) << "\t"
			// 		<< Pv_intg(1) << "\t"
			// 		<< Pv_intg(2) << "\n"
			// 		;
					// << _param_mpc_thr_max.get() << "\n";

			// cout 	<< theta_k_Pv_R(6,0) << "\t"
			// 	<< theta_k_Pv_R(7,0) << "\t"
			// 	<< theta_k_Pv_R(8,0) << "\t"
			// 	<< u_k_Pv_R(2,0) << "\t"
			// 	<< theta_k_z_R(0,0) << "\t"
			// 	<< theta_k_z_R(1,0) << "\t"
			// 	<< theta_k_z_R(2,0) << "\t"
			// 	<< u_km1_z_R(0,0) << "\n";
				//<< u_k_Pv_R(1,0) << "\t"


			thrust_desired_D = thrust_desired_D + u_k_Pv_R(2,0);
			if (1) //
			{
				//cout << "Writing RCAC_data.txt" << "\t" << dt << "\n";
				ofstream RCAC_P_v("RCAC_P_v.txt", std::fstream::in | std::fstream::out | std::fstream::app);
				if (RCAC_P_v.is_open())
				{
					RCAC_P_v << ii_Pv_R << "\t"
							<< dt << "\t"
							<< z_k_Pv_R(0,0) << "\t"
							<< z_k_Pv_R(1,0) << "\t"
							<< z_k_Pv_R(2,0) << "\t"
							<< _vel_sp(0) << "\t"
							<< _vel_sp(1) << "\t"
							<< _vel_sp(2) << "\t"
							<< _vel(0) << "\t"
							<< _vel(1) << "\t"
							<< _vel(2) << "\t"
							<< theta_k_Pv_R(0,0) << "\t"
							<< theta_k_Pv_R(1,0) << "\t"
							<< theta_k_Pv_R(2,0) << "\t"
							<< theta_k_Pv_R(3,0) << "\t"
							<< theta_k_Pv_R(4,0) << "\t"
							<< theta_k_Pv_R(5,0) << "\t"
							<< theta_k_Pv_R(6,0) << "\t"
							<< theta_k_Pv_R(7,0) << "\t"
							<< theta_k_Pv_R(8,0) << "\t"
							<< u_k_Pv_R(0,0) << "\t"
							<< u_k_Pv_R(1,0) << "\t"
							<< u_k_Pv_R(2,0) << "\t"
							//
							<< _param_mpc_z_vel_p.get() << "\t"
							<< _param_mpc_z_vel_i.get() << "\t"
							<< _param_mpc_z_vel_d.get() << "\t"
							<< _param_mpc_xy_vel_p.get() << "\t"
							<< _param_mpc_xy_vel_i.get() << "\t"
							<< _param_mpc_xy_vel_d.get() << "\t"
							<< "\n";
					RCAC_P_v.close();
				}
			}


			// if (0) //
			// {
			// 	//cout << "Writing RCAC_data.txt" << "\t" << dt << "\n";
			// 	ofstream RCAC_Data("RCAC_data.txt", std::fstream::in | std::fstream::out | std::fstream::app);
			// 	if (RCAC_Data.is_open())
			// 	{
			// 		RCAC_Data << ii_R << "\t"
			// 				  << dt << "\t"
			// 				  << z_k_x_R(0, 0) << "\t"
			// 				  << phi_k_x_R(0, 0) << "\t"
			// 				  << phi_k_x_R(0, 1) << "\t"
			// 				  << phi_k_x_R(0, 2) << "\t"
			// 				  << theta_k_x_R(0, 0) << "\t"
			// 				  << theta_k_x_R(1, 0) << "\t"
			// 				  << theta_k_x_R(2, 0) << "\t"
			// 				  << u_k_x_R(0, 0) << "\t"
			// 				  << _vel_sp(0) << "\t"
			// 				  << _vel(0) << "\t"
			// 				  //
  	// 						  << z_k_y_R(0, 0) << "\t"
			// 				  << phi_k_y_R(0, 0) << "\t"
			// 				  << phi_k_y_R(0, 1) << "\t"
			// 				  << phi_k_y_R(0, 2) << "\t"
			// 				  << theta_k_y_R(0, 0) << "\t"
			// 				  << theta_k_y_R(1, 0) << "\t"
			// 				  << theta_k_y_R(2, 0) << "\t"
			// 				  << u_k_y_R(0, 0) << "\t"
			// 				  << _vel_sp(1) << "\t"
			// 				  << _vel(1) << "\t"
			// 				  //
			// 				  << z_k_z_R(0, 0) << "\t"
			// 				  << phi_k_z_R(0, 0) << "\t"
			// 				  << phi_k_z_R(0, 1) << "\t"
			// 				  << phi_k_z_R(0, 2) << "\t"
			// 				  << theta_k_z_R(0, 0) << "\t"
			// 				  << theta_k_z_R(1, 0) << "\t"
			// 				  << theta_k_z_R(2, 0) << "\t"
			// 				  << u_k_z_R(0, 0) << "\t"
			// 				  << _vel_sp(2) << "\t"
			// 				  << _vel(2) << "\t"
			// 				  //
			// 				  << _param_mpc_z_vel_p.get() << "\t"
			// 				  << _param_mpc_z_vel_i.get() << "\t"
			// 				  << _param_mpc_z_vel_d.get() << "\t"
			// 				  << _param_mpc_xy_vel_p.get() << "\t"
			// 				  << _param_mpc_xy_vel_i.get() << "\t"
			// 				  << _param_mpc_xy_vel_d.get() << "\t"
			// 				  << "\n";
			// 		RCAC_Data.close();
			// 	}
			// }
		}
		else
		{
			// thrust_desired_D = u_k_z_R(0, 0);
			// thrust_desired_D = u_k_Pv_R(2,0);
		}
		// cout << ii_R << " " << dt << " " << _vel_sp(2) << "    " << u_k_x_R(0, 0) << " " << theta_k_x_R(0, 0) << " " <<
		// theta_k_x_R(1, 0) << " " <<
		// theta_k_x_R(2, 0) << "\n";
	}


	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -_param_mpc_thr_min.get();
	float uMin = -_param_mpc_thr_max.get();

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = math::min(uMax, -10e-4f);

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * _param_mpc_z_vel_i.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), _param_mpc_thr_max.get()) * math::sign(_thr_int(2));
	}

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = _param_mpc_xy_vel_p.get() * vel_err(0) + _param_mpc_xy_vel_d.get() * _vel_dot(0) + _thr_int(0);
		thrust_desired_NE(1) = _param_mpc_xy_vel_p.get() * vel_err(1) + _param_mpc_xy_vel_d.get() * _vel_dot(1) + _thr_int(1);

		if ( (RCAC_Pv_ON) && (1) )
		{
			//thrust_desired_NE(0) = u_k_x_R(0, 0);
			//thrust_desired_NE(1) = u_k_y_R(0, 0);
			// thrust_desired_NE(0) = u_k_Pv_R(0, 0);
			// thrust_desired_NE(1) = u_k_Pv_R(1, 0);
			thrust_desired_NE(0) = thrust_desired_NE(0) + u_k_Pv_R(0, 0);
			thrust_desired_NE(1) = thrust_desired_NE(1) + u_k_Pv_R(1, 0);


		}
		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(_param_mpc_thr_max.get() * _param_mpc_thr_max.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}

		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		float arw_gain = 2.f / _param_mpc_xy_vel_p.get();

		Vector2f vel_err_lim;
		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

		// Update integral
		_thr_int(0) += _param_mpc_xy_vel_i.get() * vel_err_lim(0) * dt;
		_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
		// cout << _thr_int(2) << "\t" 
		// 	 << vel_err(2) << "\t"
		// 	 << dt << "\t"
		// 	 << stop_integral_D << "\t"
		// 	 << _param_mpc_xy_vel_i.get() << "\t"
		// 	 << _param_mpc_thr_max.get() << "\n";

		if (abs(_thr_int(2))<0.00001f)
		{
			ii_Pr_R = 0;
			ii_Pv_R = 0;
			// cout << ii_Pr_R << "\t" << ii_Pv_R << "\n" ;
		} 

	}
}

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	const float tilt_max_radians = math::radians(math::max(_param_mpc_tiltmax_air.get(), _param_mpc_man_tilt_max.get()));

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < tilt_max_radians)) {
		_constraints.tilt = tilt_max_radians;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < _param_mpc_z_vel_max_up.get())) {
		_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < _param_mpc_z_vel_max_dn.get())) {
		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < _param_mpc_xy_vel_max.get())) {
		_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	}
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();
}
