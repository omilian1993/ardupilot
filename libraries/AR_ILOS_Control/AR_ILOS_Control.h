#pragma once

/// @file    AR_ILOS_Control.h
/// @brief   I-LOS Control algorithm. This is a instance of an
/// AP_Navigation class
/*
 * Originally written by Omar Milián 2019
 * Used on paper:
 * Autopilot for a robotic boat based on an open hardware configuration
 * Omar Milián Morón, Delvis Garcia Garcia Yunier Valeriano Medina
 * https://doi.org/10.1504/IJES.2021.121086
 */
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_Common/Location.h>
// Define statics variables
#define Tm 0.02
class AR_ILOS_Control : public AP_Navigation
{
public:
    AR_ILOS_Control(AP_AHRS &ahrs)
        : _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AR_ILOS_Control(const AR_ILOS_Control &other) = delete;
    AR_ILOS_Control &operator=(const AR_ILOS_Control &) = delete;

    int32_t nav_roll_cd(void) const override { return 0; }
    float lateral_acceleration(void) const override { return 0; }

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override { return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing)); }

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override { return 0; }

    float crosstrack_error(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _crosstrack_error; }

    int32_t target_bearing_cd(void) const override;

    float turn_distance(float wp_radius) const override { return 0; }
    float turn_distance(float wp_radius, float turn_angle) const override { return 0; }
    float loiter_radius(const float loiter_radius) const override;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f) override;
    //
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction) override { radius++; }
    void update_heading_hold(int32_t navigation_heading_cd) override { t = navigation_heading_cd; }
    void update_level_flight(void) override { t++; }
    bool reached_loiter_target(void) override { return true; }
    void set_data_is_stale(void) override { t--; }
    bool data_is_stale(void) const override { return true; }

    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override
    {
        _reverse = reverse;
    }
    void set_default_kp(float kp)
    {
        kp_ILOS.set_default(kp);
    }
    AP_Float &kP_ILOS() { return kp_ILOS; }
    void set_default_ki(float ki)
    {
        ki_ILOS.set_default(ki);
    }
    AP_Float &kI_ILOS() { return ki_ILOS; }

private:
    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd, _nav_bearing;
    AP_Float _loiter_bank_limit;
    AP_Float kp_ILOS;
    AP_Float ki_ILOS;
    float delta;
    bool _reverse = false, next = true;
    float get_yaw();
    float get_yaw_sensor();
    float _crosstrack_error;
    float aK = 0, td = 0, st = 0, et = 0, yint = 0, yintd = 0, yintda = 0, yintdao = 0, yintdaa = 0, yintdo = 0, yinto = 0, t;
};
