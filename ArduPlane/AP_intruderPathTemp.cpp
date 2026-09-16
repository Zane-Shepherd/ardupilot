#include "AP_intruderPathTemp.h"

Location intruder_loc;

bool build_intruder_path_start(const Location &first_home_location,
								Location &path_start)
{
	if (!first_home_location.initialised()) {
		return false;
	}

	path_start = first_home_location;
	path_start.offset(500.0f, 500.0f);
	path_start.offset_up_m(500.0f);
	return path_start.check_latlng();
}

void update_intruder_path(Location &path_location, float dt)
{
	path_location.offset(0.0f, 10.0f * dt);
	intruder_loc = path_location;
}
