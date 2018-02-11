
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 1)

set(config_module_list
	#
	# Board support modules
	#
	drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	drivers/magnetometer
	drivers/telemetry

	drivers/airspeed
	drivers/batt_smbus
	drivers/blinkm
	drivers/boards
	drivers/camera_trigger
	drivers/device
	drivers/gps
	drivers/imu/adis16448
	drivers/imu/bmi160
	drivers/imu/l3gd20
	drivers/imu/lsm303d
	drivers/imu/mpu6000
	drivers/imu/mpu9250
	drivers/irlock
	drivers/led
	drivers/mkblctrl
	drivers/oreoled
	drivers/protocol_splitter
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/rgbled
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/tap_esc
	drivers/vmount
	modules/sensors

	#
	# System commands
	#
	systemcmds

	#
	# Testing
	#
	drivers/distance_sensor/sf0x/sf0x_tests
	drivers/test_ppm
	lib/controllib/controllib_test
	#lib/rc/rc_tests
	modules/commander/commander_tests
	modules/mavlink/mavlink_tests
	modules/mc_pos_control/mc_pos_control_tests
	modules/uORB/uORB_tests

	#
	# General system control
	#
	modules/camera_feedback
	modules/commander
	modules/events
	modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/landing_target_estimator
	modules/local_position_estimator
	modules/position_estimator_inav

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/gnd_att_control
	modules/gnd_pos_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger
	modules/sdlog2

	#
	# Library modules
	#
	modules/dataman
	modules/systemlib
	modules/systemlib/param
	modules/uORB

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/led
	lib/mathlib
	lib/mixer
	lib/rc
	lib/terrain_estimation
	lib/tunes
	lib/version

	#
	# OBC challenge
	#
	examples/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# Segway
	#
	examples/segway

	#
	# Demo apps
	#

	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	examples/fixedwing_control

	# Hardware test
	examples/hwtest
)
