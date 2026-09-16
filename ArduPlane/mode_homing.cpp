#include "mode.h"
#include "Plane.h"
#include "AP_intruderPathTemp.h"

bool ModeHoming::_enter()
{
    if (!build_intruder_path_start(plane.home, intruder_path_location)) {
        return false;
    }

    plane.guided_throttle_passthru = false;
    plane.L1_controller.set_lateral_acceleration(0.0f);
    return true;
}

void ModeHoming::_exit()
{
    plane.L1_controller.set_lateral_acceleration(0.0f);
}

// terget altitude regreshes at 10Hz, so it has a separate call function
void ModeHoming::update_target_altitude()
{
    plane.set_relative_target_altitude(target_altitude_cm);
}

void ModeHoming::update()
{
    update_intruder_path(intruder_path_location, plane.G_Dt);

    // Vector3f interceptor_velocity;
    Quaternion interceptor_attitude;
    AP_LOSRate::Acceleration los_accel{};
    if (!plane.ahrs.get_quaternion(interceptor_attitude)) {
        plane.los_rate.reset();
        plane.L1_controller.set_lateral_acceleration(0.0f);
    } else {
        const Vector3f intruder_position = plane.current_loc.get_distance_NED(intruder_loc);
        los_accel = plane.los_rate.update(
            intruder_position,
            interceptor_attitude,
            plane.G_Dt);
        plane.L1_controller.set_lateral_acceleration(los_accel.valid ? los_accel.a_h : 0.0f);
    }
    // Set the altitude target in cm from the vertical acceleration in m/s^2.
    target_altitude_cm = los_accel.valid ? plane.current_loc.alt +
        los_accel.a_v * plane.G_Dt * plane.G_Dt * plane.G_Dt * 100 : 0.0f;

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}
