#include "mode.h"
#include "Plane.h"
#include "AP_intruderPathTemp.h"

bool ModeHoming::_enter()
{
    plane.guided_throttle_passthru = false;
    plane.L1_controller.set_lateral_acceleration(0.0f);
    return true;
}

void ModeHoming::_exit()
{
    plane.L1_controller.set_lateral_acceleration(0.0f);
}

void ModeHoming::update()
{
    Vector3f interceptor_velocity;
    Quaternion interceptor_attitude;
    if (!plane.ahrs.get_velocity_NED(interceptor_velocity) ||
        !plane.ahrs.get_quaternion(interceptor_attitude)) {
        plane.los_rate.reset();
        plane.L1_controller.set_lateral_acceleration(0.0f);
    } else {
        const Vector3f intruder_position = plane.current_loc.get_distance_NED(intruder_loc);
        const AP_LOSRate::Acceleration los_accel = plane.los_rate.update(
            Vector3f(),
            intruder_position,
            interceptor_velocity,
            interceptor_attitude,
            plane.G_Dt);
        plane.L1_controller.set_lateral_acceleration(los_accel.valid ? los_accel.a_h : 0.0f);
    }

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}
