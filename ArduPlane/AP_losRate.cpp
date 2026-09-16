// LOS angular-rate calculation for an interceptor and intruder in NED.

#include "AP_losRate.h"

AP_LOSRate::Acceleration AP_LOSRate::update(const Vector3f &intruder_position,
                                            const Quaternion &interceptor_attitude,
                                            const float dt)
{
	Acceleration acceleration;
	const Vector3f r_vector = intruder_position;

	if (!_have_previous_position) {
		_previous_r_vector = r_vector;
		_have_previous_position = true;
		return acceleration;
	}

	if (!(dt > 0.0f) || !isfinite(dt)) {
		return acceleration;
	}

	const float range_squared = r_vector.length_squared();
	if (!(range_squared > FLT_EPSILON) || !isfinite(range_squared) ||
		!isfinite(interceptor_attitude.length_squared())) {
		_previous_r_vector = r_vector;
		return acceleration;
	}

	const Vector3f relative_velocity = (r_vector - _previous_r_vector) / dt;
	const Vector3f omega_earth = r_vector % relative_velocity / range_squared;
	Vector3f omega_body = omega_earth;
	interceptor_attitude.earth_to_body(omega_body);

	const float horizontal_rate = omega_body.z;
	const float vertical_rate = omega_body.y;
	const float relative_speed = relative_velocity.length();
	acceleration.valid = isfinite(horizontal_rate) &&
						 isfinite(vertical_rate) &&
						 isfinite(relative_speed);

	_previous_r_vector = r_vector;
	if (acceleration.valid) {
		acceleration.a_h = 3.0f * horizontal_rate * relative_speed;
		acceleration.a_v = 3.0f * vertical_rate * relative_speed;
	}
	return acceleration;
}

void AP_LOSRate::reset()
{
	_previous_r_vector.zero();
	_have_previous_position = false;
}
