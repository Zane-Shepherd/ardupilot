#pragma once

#include <AP_Common/Location.h>

extern Location intruder_loc;

bool build_intruder_path_start(const Location &first_home_location,
                               Location &path_start);
void update_intruder_path(Location &path_location, float dt);