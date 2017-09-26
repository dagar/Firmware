/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file navigation.h
 * Definition of a mission consisting of mission items.
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(MEMORY_CONSTRAINED_SYSTEM)
#  define NUM_MISSIONS_SUPPORTED 50
#elif defined(__PX4_POSIX)
#  define NUM_MISSIONS_SUPPORTED (UINT16_MAX-1) // This is allocated as needed.
#else
#  define NUM_MISSIONS_SUPPORTED 2000 // This allocates a file of around 181 kB on the SD card.
#endif

#define NAV_EPSILON_POSITION	0.001f	/**< Anything smaller than this is considered zero */

/* compatible to mavlink MAV_CMD */
enum NAV_CMD {
	NAV_CMD_IDLE = 0,
	NAV_CMD_WAYPOINT = 16,
	NAV_CMD_LOITER_UNLIMITED = 17,
	NAV_CMD_LOITER_TIME_LIMIT = 19,
	NAV_CMD_RETURN_TO_LAUNCH = 20,
	NAV_CMD_LAND = 21,
	NAV_CMD_TAKEOFF = 22,
	NAV_CMD_LOITER_TO_ALT = 31,
	NAV_CMD_DO_FOLLOW_REPOSITION = 33,
	NAV_CMD_ROI = 80,
	NAV_CMD_VTOL_TAKEOFF = 84,
	NAV_CMD_VTOL_LAND = 85,
	NAV_CMD_DELAY = 93,
	NAV_CMD_DO_JUMP = 177,
	NAV_CMD_DO_CHANGE_SPEED = 178,
	NAV_CMD_DO_SET_SERVO = 183,
	NAV_CMD_DO_LAND_START = 189,
	NAV_CMD_DO_SET_ROI = 201,
	NAV_CMD_DO_DIGICAM_CONTROL = 203,
	NAV_CMD_DO_MOUNT_CONFIGURE = 204,
	NAV_CMD_DO_MOUNT_CONTROL = 205,
	NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
	NAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
	NAV_CMD_SET_CAMERA_MODE = 530,
	NAV_CMD_IMAGE_START_CAPTURE = 2000,
	NAV_CMD_IMAGE_STOP_CAPTURE = 2001,
	NAV_CMD_DO_TRIGGER_CONTROL = 2003,
	NAV_CMD_VIDEO_START_CAPTURE = 2500,
	NAV_CMD_VIDEO_STOP_CAPTURE = 2501,
	NAV_CMD_DO_VTOL_TRANSITION = 3000,
	NAV_CMD_INVALID = UINT16_MAX /* ensure that casting a large number results in a specific error */
};

/* compatible to mavlink MAV_CMD */
enum NAV_FRAME {
	NAV_FRAME_GLOBAL = 0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
	NAV_FRAME_LOCAL_NED = 1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
	NAV_FRAME_MISSION = 2, /* NOT a coordinate frame, indicates a mission command. | */
	NAV_FRAME_GLOBAL_RELATIVE_ALT = 3, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
	NAV_FRAME_LOCAL_ENU = 4, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
	NAV_FRAME_GLOBAL_INT = 5, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
	NAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
	NAV_FRAME_LOCAL_OFFSET_NED = 7, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
	NAV_FRAME_BODY_NED = 8, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
	NAV_FRAME_BODY_OFFSET_NED = 9, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
	NAV_FRAME_GLOBAL_TERRAIN_ALT = 10, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
	NAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
	NAV_FRAME_ENUM_END = 12, /*  | */
};

enum ORIGIN {
	ORIGIN_MAVLINK = 0,
	ORIGIN_ONBOARD
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Position setpoint in local coordinates.
 *
 * This is the position the MAV is heading towards. If it is of type loiter,
 * the MAV is circling around it with the given loiter radius in meters.
 */
struct navigator_item_s {
	double lat;			/**< latitude in degrees				*/
	double lon;			/**< longitude in degrees				*/
	float altitude;				/**< altitude in meters	(AMSL)			*/
	float x; /**< local position x in meters */
	float y; /**< local position y in meters */
	float z; /**< local position z in meters */
	union {
		struct {
			union {
				float time_inside; /**< time that the MAV should stay inside the radius before advancing in seconds */
				float pitch_min; /**< minimal pitch angle for fixed wing takeoff waypoints */
			};
			float acceptance_radius; /**< default radius in which the mission is accepted as reached in meters */
			float loiter_radius; /**< loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
			float yaw; /**< in radians NED -PI..+PI, NAN means don't change yaw		*/
			float _x; /**< padding */
			float _y; /**< padding */
			float _z; /**< padding */
		};
		float params[7];				/**< array to store mission command values for MAV_FRAME_MISSION ***/
	};
	uint16_t nav_cmd; /**< navigation command	*/
	int16_t do_jump_mission_index; /**< index where the do jump will go to                 */
	uint16_t do_jump_repeat_count; /**< how many times do jump needs to be done            */
	uint16_t do_jump_current_count; /**< count how many times the jump has been done	*/
	struct {
		uint16_t frame : 4, /**< mission frame ***/
			 origin : 3, /**< how the mission item was generated */
			 loiter_exit_xtrack : 1, /**< exit xtrack location: 0 for center of loiter wp, 1 for exit location */
			 force_heading : 1, /**< heading needs to be reached ***/
			 autocontinue : 1, /**< true if next waypoint should follow after this one */
			 disable_mc_yaw : 1, /**< weathervane mode */
			 vtol_back_transition : 1; /**< part of the vtol back transition sequence */
	};
	//TODO what else is needed? VTOL?
};

/**
 * Global position setpoint in WGS84 coordinates.
 *
 * This is the position the MAV is heading towards. If it of type loiter,
 * the MAV is circling around it with the given loiter radius in meters.
 *
 * Corresponds to one of the DM_KEY_WAYPOINTS_OFFBOARD_* dataman items
 */
#pragma pack(push, 1)
struct mission_item_s {
	union {
		struct {
			union {
				float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
				float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
				float circle_radius;		/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
			};
			float acceptance_radius;	/**< default radius in which the mission is accepted as reached in meters */
			float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
			float yaw;					/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
			union {
				float lat;
				int32_t lat_int;
				float x;
			};
			union {
				float lon;
				int32_t lon_int;
				float y;
			};
			union {
				float altitude;		/**< altitude in meters	(AMSL)			*/
				float z;
			};
		};
		float params[7];				/**< array to store mission command values for MAV_FRAME_MISSION ***/
	};
	uint16_t nav_cmd;					/**< navigation command					*/
	int16_t do_jump_mission_index;		/**< index where the do jump will go to                 */
	uint16_t do_jump_repeat_count;		/**< how many times do jump needs to be done            */
	union {
		uint16_t do_jump_current_count;		/**< count how many times the jump has been done	*/
		uint16_t vertex_count;			/**< Polygon vertex count (geofence)	*/
	};
	struct {
		uint16_t frame : 4,					/**< mission frame */
			 origin : 3,						/**< how the mission item was generated */
			 loiter_exit_xtrack : 1,			/**< exit xtrack location: 0 for center of loiter wp, 1 for exit location */
			 force_heading : 1,				/**< heading needs to be reached */
			 altitude_is_relative : 1,		/**< true if altitude is relative from start point	*/
			 autocontinue : 1,				/**< true if next waypoint should follow after this one */
			 disable_mc_yaw : 1,				/**< weathervane mode */
			 vtol_back_transition : 1;		/**< part of the vtol back transition sequence */
	};
};

/**
 * dataman housekeeping information for a specific item.
 * Corresponds to the first dataman entry of DM_KEY_FENCE_POINTS and DM_KEY_SAFE_POINTS
 */
struct mission_stats_entry_s {
	uint16_t num_items;			/**< total number of items stored (excluding this one) */
	uint16_t update_counter;			/**< This counter is increased when (some) items change (this can wrap) */
};

/**
 * Geofence vertex point.
 * Corresponds to the DM_KEY_FENCE_POINTS dataman item
 */
struct mission_fence_point_s {
	double lat;
	double lon;
	float alt;
	uint16_t nav_cmd;				/**< navigation command (one of MAV_CMD_NAV_FENCE_*) */
	union {
		uint16_t vertex_count;			/**< number of vertices in this polygon */
		float circle_radius;			/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
	};
	uint8_t frame;					/**< MAV_FRAME */
};

/**
 * Save Point (Rally Point).
 * Corresponds to the DM_KEY_SAFE_POINTS dataman item
 */
struct mission_save_point_s {
	double lat;
	double lon;
	float alt;
	uint8_t frame;					/**< MAV_FRAME */
};

#pragma pack(pop)
#include <uORB/topics/mission.h>

/**
 * @}
 */


#endif
