
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5x
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	TESTING
	UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS6
		TEL2:/dev/ttyS4
		TEL3:/dev/ttyS2
		GPS2:/dev/ttyS0

	DRIVERS
		adc
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		#heater
		#imu # all available imu drivers
		#imu/bmi088
		#imu/mpu6000
		imu/st/ism330dlc
		irlock
		lights/blinkm
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		#md25
		mkblctrl
		optical_flow # all available optical flow drivers
		pca9685
		power_monitor/ina226
		#protocol_splitter
		pwm_input
		pwm_out_sim
		px4fmu
		px4io
		rc_input
		roboclaw
		safety_button
		tap_esc
		telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		uavcan

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		rover_pos_control
		sensors
		sih
		vmount
		vtol_att_control
		airspeed_selector

	SYSTEMCMDS
		bl_update
		config
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue

	EXAMPLES
		bottle_drop # OBC challenge
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		hwtest # Hardware test
		#matlab_csv_serial
		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		rover_steering_control # Rover example app
		uuv_example_app
	)
