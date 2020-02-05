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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <px4_module_params.h>
#pragma once

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl : public ModuleParams
{
public:

	PositionControl(ModuleParams *parent);
	~PositionControl() = default;

	/**
	 *	Overwrites certain parameters.
	 *	Overwrites are required for unit-conversion.
	 *	This method should only be called if parameters
	 *	have been updated.
	 */
	void overwriteParams();

	/**
	 * Update the current vehicle state.
	 * @param PositionControlStates structure
	 */
	void updateState(const PositionControlStates &states);

	/**
	 * Update the desired setpoints.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 * @return true if setpoint has updated correctly
	 */
	bool updateSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Set constraints that are stricter than the global limits.
	 * @param constraints a PositionControl structure with supported constraints
	 */
	void updateConstraints(const vehicle_constraints_s &constraints);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt the delta-time
	 */
	void generateThrustYawSetpoint(const float dt);

	/**
	 * 	Set the integral term in xy to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralXY() { _thr_int(0) = _thr_int(1) = 0.0f; }

	/**
	 * 	Set the integral term in z to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralZ() { _thr_int(2) = 0.0f; }

	/**
	 * 	Get the
	 * 	@see _thr_sp
	 * 	@return The thrust set-point member.
	 */
	const matrix::Vector3f &getThrustSetpoint() { return _thr_sp; }

	/**
	 * 	Get the
	 * 	@see _yaw_sp
	 * 	@return The yaw set-point member.
	 */
	const float &getYawSetpoint() { return _yaw_sp; }

	/**
	 * 	Get the
	 * 	@see _yawspeed_sp
	 * 	@return The yawspeed set-point member.
	 */
	const float &getYawspeedSetpoint() { return _yawspeed_sp; }

	/**
	 * 	Get the
	 * 	@see _vel_sp
	 * 	@return The velocity set-point that was executed in the control-loop. Nan if velocity control-loop was skipped.
	 */
	const matrix::Vector3f getVelSp()
	{
		matrix::Vector3f vel_sp{};

		for (int i = 0; i <= 2; i++) {
			if (_ctrl_vel[i]) {
				vel_sp(i) = _vel_sp(i);

			} else {
				vel_sp(i) = NAN;
			}
		}

		return vel_sp;
	}

	/**
	 * 	Get the
	 * 	@see _pos_sp
	 * 	@return The position set-point that was executed in the control-loop. Nan if the position control-loop was skipped.
	 */
	const matrix::Vector3f getPosSp()
	{
		matrix::Vector3f pos_sp{};

		for (int i = 0; i <= 2; i++) {
			if (_ctrl_pos[i]) {
				pos_sp(i) = _pos_sp(i);

			} else {
				pos_sp(i) = NAN;
			}
		}

		return pos_sp;
	}

	/**
	 * 	Get the
	 * 	@see z_k_Pr_R
	 * 	@return The z variable used by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_z()
	{
		matrix::Vector3f RCAC_z{};

		for (int i = 0; i <= 2; i++) {
			RCAC_z(i) = z_k_Pr_R(i,0);
		}

		return RCAC_z;
	}

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_u()
	{
		matrix::Vector3f RCAC_u{};

		for (int i = 0; i <= 2; i++) {
			RCAC_u(i) = u_k_Pr_R(i,0);
		}

		return RCAC_u;
	}

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_theta()
	{
		matrix::Vector3f RCAC_theta{};

		for (int i = 0; i <= 2; i++) {
			RCAC_theta(i) = theta_k_Pr_R(i,0);
		}

		return RCAC_theta;
	}

	/**
	 * 	Get the
	 * 	@see z_k_Pv_R
	 * 	@return The z variable used by RCAC in the PID velocity controller
	 */
	const matrix::Vector3f get_RCAC_vel_z()
	{
		matrix::Vector3f RCAC_z{};

		for (int i = 0; i <= 2; i++) {
			RCAC_z(i) = z_k_vel(i); //z_k_Pv_R(i,0);
		}

		return RCAC_z;
	}

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_vel_u()
	{
		matrix::Vector3f RCAC_u{};

		for (int i = 0; i <= 2; i++) {
			RCAC_u(i) = u_k_vel(i); //u_k_Pv_R(i,0);
		}

		return RCAC_u;
	}

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Matrix<float, 9,1> get_RCAC_vel_theta()
	{
		matrix::Matrix<float, 9,1> RCAC_theta{};

		for (int i = 0; i <= 2; i++) {
			//RCAC_theta(i,0) = theta_k_Pv_R(i,0);
			RCAC_theta(i,0) = theta_k_vel_x(i);
			RCAC_theta(i+3,0) = theta_k_vel_y(i);
			RCAC_theta(i+6,0) = theta_k_vel_z(i);
		}

		return RCAC_theta;
	}

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC position controller
	 */
	const int &get_RCAC_pos_ii() { return ii_Pr_R; }

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC velocity controller
	 */
	const int &get_RCAC_vel_ii() { return ii_Pv_R; }

	/**
	 * 	Set the RCAC position switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_pos_switch(float switch_RCAC)
	{
		RCAC_Pr_ON = 1;
		if (switch_RCAC<0.0f) {
			RCAC_Pr_ON = 0;
		}
	}

	/**
	 * 	Set the RCAC velocity switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_vel_switch(float switch_RCAC)
	{
		RCAC_Pv_ON = 1;
		if (switch_RCAC<0.0f) {
			RCAC_Pv_ON = 0;
		}
	}

	/**
	 * 	Set the PID scaling factor.
	 * 	@see _thr_int
	 */
	void set_PID_pv_factor(float PID_factor)
	{
		alpha_PID = 1;
		if (PID_factor<0.0f) {
			alpha_PID = 0.5;
		}
	}

	/**
	 * 	Get the
	 * 	@see RCAC_Pr_ON
	 * 	@return Get RCAC pos controller switch
	 */
	const bool &get_RCAC_pos_switch() {return RCAC_Pr_ON;}

	/**
	 * 	Get the
	 * 	@see RCAC_Pr_ON
	 * 	@return Get RCAC vel controller switch
	 */
	const bool &get_RCAC_vel_switch() {return RCAC_Pv_ON;}

	/**
	 * 	Reset RCAC variables
	 * 	@see _thr_int
	 */
	void resetRCAC()
	{
		// P_vel_x = eye<float, 3>() * 0.0010;
		// P_vel_y = eye<float, 3>() * 0.0010;
		// P_vel_z = eye<float, 3>() * 0.0010;
		// P_Pr_R = eye<float, 3>() * 0.010 * alpha_P;

		P_vel_x.setZero();
		P_vel_y.setZero();
		P_vel_z.setZero();
		P_Pr_R.setZero();
		for (int i = 0; i <= 2; i++) {
			P_vel_x(i,i) = 0.001;
			P_vel_y(i,i) = 0.001;
			P_vel_z(i,i) = 0.001;
			P_Pr_R(i,i) = 0.01;
		}
		phi_k_vel_x.setZero();
		phi_k_vel_y.setZero();
		phi_k_vel_z.setZero();
		phi_km1_vel_x.setZero();
		phi_km1_vel_y.setZero();
		phi_km1_vel_z.setZero();
		theta_k_vel_x.setZero();
		theta_k_vel_y.setZero();
		theta_k_vel_z.setZero();
		u_k_vel.setZero();
		z_k_vel.setZero();

		phi_km1_Pr_R.setZero();
		theta_k_Pr_R.setZero();
		z_k_Pr_R.setZero();
		z_km1_Pr_R.setZero();
		u_k_Pr_R.setZero();
		u_km1_Pr_R.setZero();

		ii_Pr_R = 0;
		ii_Pv_R = 0;
	}
protected:

	void updateParams() override;

private:
	/**
	 * Maps setpoints to internal-setpoints.
	 * @return true if mapping succeeded.
	 */
	bool _interfaceMapping();

	void _positionController(); /** applies the P-position-controller */
	void _velocityController(const float &dt); /** applies the PID-velocity-controller */
	void _setCtrlFlag(bool value); /**< set control-loop flags (only required for logging) */

	matrix::Vector3f _pos{}; /**< MC position */
	matrix::Vector3f _vel{}; /**< MC velocity */
	matrix::Vector3f _vel_dot{}; /**< MC velocity derivative */
	matrix::Vector3f _acc{}; /**< MC acceleration */
	float _yaw{0.0f}; /**< MC yaw */
	matrix::Vector3f _pos_sp{}; /**< desired position */
	matrix::Vector3f _vel_sp{}; /**< desired velocity */
	matrix::Vector3f _acc_sp{}; /**< desired acceleration: not supported yet */
	matrix::Vector3f _thr_sp{}; /**< desired thrust */
	float _yaw_sp{}; /**< desired yaw */
	float _yawspeed_sp{}; /** desired yaw-speed */
	matrix::Vector3f _thr_int{}; /**< thrust integral term */
	vehicle_constraints_s _constraints{}; /**< variable constraints */
	bool _skip_controller{false}; /**< skips position/velocity controller. true for stabilized mode */
	bool _ctrl_pos[3] = {true, true, true}; /**< True if the control-loop for position was used */
	bool _ctrl_vel[3] = {true, true, true}; /**< True if the control-loop for velocity was used */


	int ii_Pr_R = 0;
	bool RCAC_Pr_ON=1;
	matrix::SquareMatrix<float, 3> P_Pr_R;
	matrix::Matrix<float, 3,3> phi_k_Pr_R, phi_km1_Pr_R;
	matrix::Matrix<float, 3,1> theta_k_Pr_R;
  	matrix::Matrix<float, 3,1> z_k_Pr_R, z_km1_Pr_R,u_k_Pr_R, u_km1_Pr_R;
	matrix::SquareMatrix<float, 3> Gamma_Pr_R;

	// const TODO: make really const.
	matrix::SquareMatrix<float, 3> I3, N1_Pr;

	int ii_Pv_R = 0;
	bool RCAC_Pv_ON=1;
	bool _rcac_logging = true; /**< True if logging the aircraft state variable */ //TODO: MAV integration

	// matrix::SquareMatrix<float, 9> P_Pv_R;
	// matrix::Matrix<float, 3,9> phi_k_Pv_R, phi_km1_Pv_R;
	// matrix::Matrix<float, 9,1> theta_k_Pv_R,theta_k_Pv_PID;
  	// matrix::Matrix<float, 3,1> z_k_Pv_R, z_km1_Pv_R,u_k_Pv_R, u_km1_Pv_R;
	// matrix::SquareMatrix<float, 3> Gamma_Pv_R, N1_Pv;

	matrix::SquareMatrix<float, 3> P_vel_x,P_vel_y,P_vel_z;
	matrix::Matrix<float, 1,3> phi_k_vel_x, phi_km1_vel_x;
	matrix::Matrix<float, 1,3> phi_k_vel_y, phi_km1_vel_y;
	matrix::Matrix<float, 1,3> phi_k_vel_z, phi_km1_vel_z;
	matrix::Vector3f theta_k_vel_x, theta_k_vel_y, theta_k_vel_z;
  	matrix::Vector3f z_k_vel, z_km1_vel, u_k_vel, u_km1_vel;
	matrix::Vector3f N1_vel, Gamma_vel;
	matrix::Matrix<float, 1,1> dummy1,dummy2,dummy3;

	float alpha_PID = 1.0f;

	matrix::Vector3f Pv_intg;

    DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)
		_param_mpc_tiltmax_air, // maximum tilt for any position controlled mode in degrees
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>)
		_param_mpc_man_tilt_max, // maximum til for stabilized/altitude mode in degrees
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_Z_VEL_P>) _param_mpc_z_vel_p,
		(ParamFloat<px4::params::MPC_Z_VEL_I>) _param_mpc_z_vel_i,
		(ParamFloat<px4::params::MPC_Z_VEL_D>) _param_mpc_z_vel_d,
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P>) _param_mpc_xy_vel_p,
		(ParamFloat<px4::params::MPC_XY_VEL_I>) _param_mpc_xy_vel_i,
		(ParamFloat<px4::params::MPC_XY_VEL_D>) _param_mpc_xy_vel_d
            )

    void init_RCAC();
};
