/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include <lib/mixer/mixer.h>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <vtol_att_control/vtol_type.h>

#include <AttitudeControl.hpp>

/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams
{
public:
	MulticopterAttitudeControl();

	virtual ~MulticopterAttitudeControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterAttitudeControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();
	void		vehicle_status_poll();

	float		throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void		generate_attitude_setpoint(float dt, bool reset_yaw_sp);

	/**
	 * Attitude controller.
	 */
	void		control_attitude();

	AttitudeControl _attitude_control; /**< class for attitude control calculations */

	uORB::Subscription _v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint subscription */
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */

	int		_vehicle_attitude_sub{-1};	/**< vehicle attitude data subscription */

	orb_advert_t	_v_rates_sp_pub{nullptr};		/**< rate setpoint publication */
	orb_advert_t	_vehicle_attitude_setpoint_pub{nullptr};

	orb_id_t	_attitude_sp_id{nullptr};	/**< pointer to correct attitude setpoint uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
	struct vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	struct vehicle_land_detected_s		_vehicle_land_detected {};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,

		(ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,	/**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,	/**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,		/**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover, /**< throttle at which vehicle is at hover equilibrium */
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,		/**< throttle curve behavior */

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,		/**< scaling factor from stick to yaw rate */

		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode
	)

	bool _is_tailsitter{false};

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */
	float _man_tilt_max;			/**< maximum tilt allowed for manual flight [rad] */

};

