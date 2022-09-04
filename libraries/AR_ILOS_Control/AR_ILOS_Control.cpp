#include <AP_HAL/AP_HAL.h>
#include "AR_ILOS_Control.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;
// table of user settable parameters
const AP_Param::GroupInfo AR_ILOS_Control::var_info[] = {
	// @Param: KP_ILOS
	// @DisplayName: Proportional gain of I-LOS algoritmh
	// @Description: KP_ILOS=1/DELTA
	// @Range: 0.01 1
	// @User: Advance
	AP_GROUPINFO("KP_ILOS", 0, AR_ILOS_Control, kp_ILOS, 0.5f),

	// @Param: KI_ILOS
	// @DisplayName: Integral gain of I-LOS algoritmh
	// @Description: KI_ILOS integral gain
	// @Range: 0.01 1
	// @User: Advance
	AP_GROUPINFO("KI_ILOS", 1, AR_ILOS_Control, ki_ILOS, 0.25f),

	AP_GROUPEND};

/*
  Wrap AHRS yaw if in reverse - radians
 */
float AR_ILOS_Control::get_yaw()
{
	if (_reverse)
	{
		return wrap_PI(M_PI + _ahrs.yaw);
	}
	return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
float AR_ILOS_Control::get_yaw_sensor()
{
	if (_reverse)
	{
		return wrap_180_cd(18000 + _ahrs.yaw_sensor);
	}
	return _ahrs.yaw_sensor;
}

int32_t AR_ILOS_Control::target_bearing_cd(void) const
{
	return wrap_180_cd(_target_bearing_cd);
}

// update I-LOS control for waypoint navigation
void AR_ILOS_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{ // yaw, yintda, yintdao, yintdaa, yintdo, yinto;
	struct Location _current_loc;
	if (_ahrs.get_position(_current_loc) == false)
	{
		// if no GPS loc available, maintain last nav/target_bearing
		return;
	}
	delta = (float)1.0f / kp_ILOS;

	struct Location origin;
	if (_ahrs.get_origin(origin) == false)
	{
		//	return;
	}
	Vector2f sp = origin.get_distance_NE(prev_WP);
	Vector2f ap = origin.get_distance_NE(next_WP);
	Vector2f gps = origin.get_distance_NE(_current_loc);
	//
	aK = atan2f(ap.y - sp.y, ap.x - sp.x);
	//
	_nav_bearing = aK; // LOS
	//
	td = (ap.x - sp.x) * cosf(aK) + (ap.y - sp.y) * sinf(aK);
	//
	st = (gps.x - sp.x) * cosf(aK) + (gps.y - sp.y) * sinf(aK);
	et = -(gps.x - sp.x) * sinf(aK) + (gps.y - sp.y) * cosf(aK);
	_crosstrack_error = et;
	//
	yintd = (et * delta) / (delta * delta + (et + ki_ILOS * yint) * (et + kp_ILOS * yint));
	//
	yint += Tm * yintd / 3;
	if (yint < -1.0f)
	{
		yint = -1.0f;
	}
	else if (yint > 1.0f)
	{
		yint = 1.0f;
	}
	// Other integration mode
	//     yint += ((float)et * kp_ILOS*K) * Tm;
	//             if (yint < -1.0f) {
	//                 yint = -1.0f;
	//             } else if (yint > 1.0f) {
	//                 yint = 1.0f;
	//             }
	_target_bearing_cd = degrees((-atanf(kp_ILOS * (et + ki_ILOS * yint)) + aK)) * 100; // I-LOS
	gcs().send_text(MAV_SEVERITY_NOTICE, "I-LOS Constroller.");
}
float AR_ILOS_Control::loiter_radius(const float radius) const
{
	// prevent an insane loiter bank limit
	float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
	float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

	float nominal_velocity_sea_level = 0.0f;

	float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

	if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
		is_zero(lateral_accel_sea_level))
	{
		// Missing a sane input for calculating the limit, or the user has
		// requested a straight scaling with altitude. This will always vary
		// with the current altitude, but will at least protect the airframe
		return radius * eas2tas_sq;
	}
	else
	{
		float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
		if (sea_level_radius > radius)
		{
			// If we've told the plane that its sea level radius is unachievable fallback to
			// straight altitude scaling
			return radius * eas2tas_sq;
		}
		else
		{
			// select the requested radius, or the required altitude scale, whichever is safer
			return MAX(sea_level_radius * eas2tas_sq, radius);
		}
	}
}
