#ifndef NAV_STRUCTS_H
#define NAV_STRUCTS_H
struct nav_data_t {
	struct nmea_data_t gps;
	struct sonar_data_t sonar;
	struct optical_data_t optical;
};
#endif
