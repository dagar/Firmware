# Optical flow in XYZ body frame in SI units.

uint64 timestamp                    # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                    # unique device ID for the sensor that does not change between power cycles

float32 pixel_flow_x                # accumulated optical flow in radians where a positive value is produced by a RH rotation about the X body axis
float32 pixel_flow_y                # accumulated optical flow in radians where a positive value is produced by a RH rotation about the Y body axis

uint32 error_count

uint8 quality                       # Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate               # Magnitude of maximum angular which the optical flow sensor can measure reliably
float32 min_ground_distance         # Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance         # Maximum distance from ground at which the optical flow sensor operates reliably

uint8 orientation		    # Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

uint8 ROTATION_YAW_0		= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_YAW_45		= 1 # MAV_SENSOR_ROTATION_YAW_45
uint8 ROTATION_YAW_90		= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_YAW_135		= 3 # MAV_SENSOR_ROTATION_YAW_135
uint8 ROTATION_YAW_180		= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_YAW_225		= 5 # MAV_SENSOR_ROTATION_YAW_225
uint8 ROTATION_YAW_270		= 6 # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_YAW_315		= 7 # MAV_SENSOR_ROTATION_YAW_315

uint8 ROTATION_FORWARD_FACING	= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_RIGHT_FACING	= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_BACKWARD_FACING	= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_LEFT_FACING	= 6 # MAV_SENSOR_ROTATION_YAW_270

uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270
