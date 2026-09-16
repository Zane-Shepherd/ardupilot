#pragma once

#include <AP_Math/AP_Math.h>

class AP_LOSRate {
public:
    struct Acceleration {
        float a_h = 0.0f;
        float a_v = 0.0f;
        bool valid = false;
    };

    Acceleration update(const Vector3f &interceptor_position,
                        const Vector3f &intruder_position,
                        const Vector3f &interceptor_velocity,
                        const Quaternion &interceptor_attitude,
                        float dt);

    void reset();

private:
    Vector3f _previous_r_vector;
    bool _have_previous_position = false;
};