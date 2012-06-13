#if GPS

#if defined(TINY_GPS)
#include "tinygps.h"
#endif

#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD) || defined(TINY_GPS)

  struct PID_PARAM {
    float kP;
    float kI;
    float kD;
    float Imax;
  };
  PID_PARAM posholdPID;
  PID_PARAM poshold_ratePID;
  PID_PARAM navPID;

  // AC_PID.h & AC_PID.cpp
  class AC_PID {
  public:
    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    int32_t get_p(int32_t error,PID_PARAM* pid);
    int32_t get_i(int32_t error, float* dt,PID_PARAM* pid);
    int32_t get_d(int32_t error, float* dt,PID_PARAM* pid);
    
    /// Reset the PID integrator
    ///
    void reset();

    float get_integrator() const { return _integrator; }
    void set_integrator(float i) { _integrator = i; }
    
  private:
    float _integrator; ///< integrator value
    int32_t _last_input; ///< last input for derivative
    float _last_derivative; ///< last derivative for low-pass filter
    float _output;
    float _derivative;
    
    /// Low pass filter cut frequency for derivative calculation.
    ///
    static const float _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3
  };
  int32_t AC_PID::get_p(int32_t error,PID_PARAM* pid) {
    return (float)error * pid->kP;
  }
  
  int32_t AC_PID::get_i(int32_t error, float* dt,PID_PARAM* pid) {
    _integrator += ((float)error * pid->kI) * *dt;
    if (_integrator < -pid->Imax) {
      _integrator = -pid->Imax;
    } else if (_integrator > pid->Imax) {
      _integrator = pid->Imax;
    }
    return _integrator;
  }
  
  int32_t AC_PID::get_d(int32_t input, float* dt,PID_PARAM* pid) {
    _derivative = (input - _last_input) / *dt;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    _derivative = _last_derivative + (*dt / ( _filter + *dt)) * (_derivative - _last_derivative);
    // update state
    _last_input = input;
    _last_derivative    = _derivative;
    // add in derivative component
    return pid->kD * _derivative;
  }

  void AC_PID::reset() {
    _integrator = 0;
    _last_input = 0;
    _last_derivative = 0;
  }

  #define _X 1
  #define _Y 0

  /****************** PI and PID controllers for GPS ********************/ //32938 -> 33160
  AC_PID pi_poshold[2];
  AC_PID pid_poshold_rate[2];
  AC_PID pid_nav[2];

  #define RADX100                    0.000174532925  
  #define CROSSTRACK_GAIN            1
  #define NAV_SPEED_MIN              100    // cm/sec
  #define NAV_SPEED_MAX              300    // cm/sec
  #define NAV_SLOW_NAV               true
  #define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)

  static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
  static uint16_t GPS_wp_radius    = GPS_WP_RADIUS;
  static int16_t actual_speed[2] = {0,0};
  static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

  // The difference between the desired rate of travel and the actual rate of travel
  // updated after GPS read - 5-10hz
  static int16_t rate_error[2];
  static int32_t error[2];

  //Currently used WP
  static int32_t GPS_WP[2];

  ////////////////////////////////////////////////////////////////////////////////
  // Location & Navigation
  ////////////////////////////////////////////////////////////////////////////////
  // This is the angle from the copter to the "next_WP" location in degrees * 100
  static int32_t target_bearing;
  ////////////////////////////////////////////////////////////////////////////////
  // Crosstrack
  ////////////////////////////////////////////////////////////////////////////////
  // deg * 100, The original angle to the next_WP when the next_WP was set
  // Also used to check when we pass a WP
  static int32_t original_target_bearing;
  // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
  static int16_t crosstrack_error;
  ////////////////////////////////////////////////////////////////////////////////
  // The location of the copter in relation to home, updated every GPS read (1deg - 100)
  // static int32_t home_to_copter_bearing; /* unused */
  // distance between plane and home in cm
  // static int32_t home_distance; /* unused */
  // distance between plane and next_WP in cm
  static uint32_t wp_distance;
  
  // used for slow speed wind up when start navigation;
  static uint16_t waypoint_speed_gov;

  ////////////////////////////////////////////////////////////////////////////////////
  // moving average filter variables
  //
  
  #define GPS_FILTER_VECTOR_LENGTH 5
  
  static uint8_t GPS_filter_index = 0;
  static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
  static int32_t GPS_filter_sum[2];
  static int32_t GPS_read[2];
  static int32_t GPS_filtered[2];
  static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
  static uint16_t fraction3[2];
#endif

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t	nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t  nav_takeoff_bearing;


#if defined(I2C_GPS)

/////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS helper functions
//
// Send a command to the I2C GPS module, first parameter command, second parameter wypoint number
void GPS_I2C_command(uint8_t command, uint8_t wp) {
  uint8_t _cmd;
    
  _cmd = (wp << 4) + command;
  
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
  i2c_write(I2C_GPS_COMMAND);
  i2c_write(_cmd);
}
#endif 

void GPS_NewData() {
  uint8_t axis;
  #if defined(I2C_GPS)
    static uint8_t _i2c_gps_status;
  
    //Do not use i2c_writereg, since writing a register does not work if an i2c_stop command is issued at the end
    //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.
  
    GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
    _i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00);                 //Get status register 
    if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
       flag.GPS_FIX = 1;
       
       if (!flag.ARMED) { flag.GPS_FIX_HOME = 0 }          // Clear home position when disarmed
       
       if (!flag.GPS_FIX_HOME && flag.ARMED) {        //if home is not set set home position to WP#0 and activate it
          GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);      //Store current position to WP#0 (this is used for RTH)
          nav_takeoff_bearing = heading;                  //Store takeof heading
          flag.GPS_FIX_HOME = 1;
       }
       if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                               //Check about new data
          if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                  //Fancy flash on GUI :D
          //Read GPS data for distance, heading and gps position 

          i2c_rep_start(I2C_GPS_ADDRESS<<1);
          i2c_write(I2C_GPS_NAV_BEARING);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

          uint8_t *varptr = (uint8_t *)&nav_bearing;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_directionToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();
          GPS_directionToHome = GPS_directionToHome / 100;  // 1deg =1000 in the reg, downsize
          if (GPS_directionToHome>180) GPS_directionToHome -= 360;

          varptr = (uint8_t *)&GPS_distanceToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();
          GPS_distanceToHome = GPS_distanceToHome / 100;      //register is in CM, we need in meter

          i2c_rep_start(I2C_GPS_ADDRESS<<1);
          i2c_write(I2C_GPS_LOCATION);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

          varptr = (uint8_t *)&GPS_coord[LAT];		// for OSD latitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_coord[LON];		// for OSD longitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&nav[LAT];		 
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          
          varptr = (uint8_t *)&nav[LON];		 
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readNak();
          
          debug1=nav[LAT];
          debug2=nav[LON];
          
          i2c_rep_start(I2C_GPS_ADDRESS<<1);
          i2c_write(I2C_GPS_GROUND_SPEED);          
          i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

          varptr = (uint8_t *)&GPS_speed;			// speed in cm/s for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_altitude;       // altitude in meters for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          //GPS_ground_course
          varptr = (uint8_t *)&GPS_ground_course;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();

          //Adjust heading when navigating
          if (flag.GPS_HOME_MODE)
          {  if ( !(_i2c_gps_status & I2C_GPS_STATUS_WP_REACHED) )
              {
          	//Tail control	
                if (NAV_CONTROLS_HEADING) {
                  if (NAV_TAIL_FIRST) {
                      magHold = nav_bearing/100-180;
                      if (magHold > 180)  magHold -= 360;
	              if (magHold < -180) magHold += 360;
                  } else {
                      magHold = nav_bearing/100;
                  }
                }
              } else {        //Home position reached
              if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
              }
          }
        }
    } else {                                                                          //We don't have a fix zero out distance and bearing (for safety reasons)
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_numSat = 0;
    }
  #endif     

  #if defined(GPS_SERIAL) || defined(TINY_GPS)
  #if defined(GPS_SERIAL)
    while (SerialAvailable(GPS_SERIAL)) {
     if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
  #elif defined(TINY_GPS)
    {
      {
      tinygps_query();
  #endif
       if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
        if (flag.GPS_FIX && GPS_numSat >= 5) {
          if (!flag.ARMED) {flag.GPS_FIX_HOME = 0;}
          if (!flag.GPS_FIX_HOME && flag.ARMED) {
            flag.GPS_FIX_HOME = 1;
            GPS_home[LAT] = GPS_coord[LAT];
            GPS_home[LON] = GPS_coord[LON];
            GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
            nav_takeoff_bearing = heading;             //save takeoff heading
          }
          //Apply moving average filter to GPS data
    #if defined(GPS_FILTERING)
          GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
          for (axis = 0; axis< 2; axis++) {
            GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
            GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
    
            // How close we are to a degree line ? its the first three digits from the fractions of degree
            // later we use it to Check if we are close to a degree line, if yes, disable averaging,
            fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;
    
            GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
            GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000); 
            GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
            GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
            if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
              if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS_coord[axis] = GPS_filtered[axis];
            }
          }
    #endif
          //dTnav calculation
          //Time for calculating x,y speed and navigation pids
          static uint32_t nav_loopTimer;
	  dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
	  nav_loopTimer = millis();
          // prevent runup from bad GPS
	  dTnav = min(dTnav, 1.0);  

          //calculate distance and bearings for gui and other stuff continously - From home to copter
          uint32_t dist;
          int32_t  dir;
          GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
          GPS_distanceToHome = dist/100;
          GPS_directionToHome = dir/100;
 
          
          //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
          GPS_calc_velocity();        
          
          if (flag.GPS_HOLD_MODE || flag.GPS_HOME_MODE){    //ok we are navigating 
            //do gps nav calculations here, these are common for nav and poshold  
            GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
            GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);

            switch (nav_mode) {
              case NAV_MODE_POSHOLD: 
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
		GPS_calc_poshold();
                break;
              case NAV_MODE_WP:
		int16_t speed = GPS_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation 
		// use error as the desired rate towards the target
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
		GPS_calc_nav_rate(speed);
	
    	        //Tail control	
                if (NAV_CONTROLS_HEADING) {
                  if (NAV_TAIL_FIRST) {
                    magHold = wrap_18000(nav_bearing-18000)/100;
                  } else {
                    magHold = nav_bearing/100;
                  }
                }
                // Are we there yet ?(within 2 meters of the destination)
		if ((wp_distance <= GPS_wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
                  nav_mode = NAV_MODE_POSHOLD;
                  if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
                } 
                break;               
            }
          } //end of gps calcs  
        }
      }
    }
  #endif

  #if defined(GPS_FROM_OSD)
    if(GPS_update) {
      if (flag.GPS_FIX && GPS_numSat > 3) {
        if (!flag.GPS_HOME_FIX) {
          flag.GPS_HOME_FIX = 1;
          GPS_home[LAT] = GPS_coord[LAT];
          GPS_home[LON] = GPS_coord[LON];
        }
        if(flag.GPS_HOLD_MODE) {
          GPS_distance(GPS_hold[LAT],GPS_hold[LON],GPS_coord[LAT],GPS_coord[LON], &GPS_distanceToHold, &GPS_directionToHold);
        else
          GPS_distance(GPS_home[LAT],GPS_home[LON],GPS_coord[LAT],GPS_coord[LON], &GPS_distanceToHome, &GPS_directionToHome);
        }
        GPS_update = 0;
    }
  #endif
}

void GPS_reset_home_position() {
  #if defined(I2C_GPS)
    //set current position as home
    GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);  //WP0 is the home position
  #else
    GPS_home[LAT] = GPS_coord[LAT];
    GPS_home[LON] = GPS_coord[LON];
  #endif
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav() {
  for(uint8_t i=0;i<2;i++) {
    GPS_angle[i]  = 0;
    nav[i] = 0;
    #if defined(I2C_GPS)
      //GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
    #else
      pi_poshold[i].reset();
      pid_poshold_rate[i].reset();
      pid_nav[i].reset();
    #endif
  }
}

//Get the relevant P I D values and set the PID controllers 
void GPS_set_pids() {
#if defined(GPS_SERIAL)  || defined(GPS_FROM_OSD) || defined(TINY_GPS)
  posholdPID.kP =   (float)conf.P8[PIDPOS]/100.0;
  posholdPID.kI =   (float)conf.I8[PIDPOS]/100.0;
  posholdPID.Imax = POSHOLD_RATE_IMAX * 100;
  
  poshold_ratePID.kP =   (float)conf.P8[PIDPOSR]/10.0;
  poshold_ratePID.kI =   (float)conf.I8[PIDPOSR]/100.0;
  poshold_ratePID.kD =   (float)conf.D8[PIDPOSR]/1000.0;
  poshold_ratePID.Imax = POSHOLD_RATE_IMAX * 100;
  
  navPID.kP =   (float)conf.P8[PIDNAVR]/10.0;
  navPID.kI =   (float)conf.I8[PIDNAVR]/100.0;
  navPID.kD =   (float)conf.D8[PIDNAVR]/1000.0;
  navPID.Imax = POSHOLD_RATE_IMAX * 100;
#endif

#if defined(I2C_GPS)
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_HOLD_P);
     i2c_write(conf.P8[PIDPOS]);
     i2c_write(conf.I8[PIDPOS]);
  
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_HOLD_RATE_P);
     i2c_write(conf.P8[PIDPOSR]);
     i2c_write(conf.I8[PIDPOSR]);
     i2c_write(conf.D8[PIDPOSR]);
  
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_NAV_P);
     i2c_write(conf.P8[PIDNAVR]);
     i2c_write(conf.I8[PIDNAVR]);
     i2c_write(conf.D8[PIDNAVR]);
  
  GPS_I2C_command(I2C_GPS_COMMAND_UPDATE_PIDS,0);
  
  uint8_t nav_flags = 0;
  if (GPS_FILTERING)          nav_flags += I2C_NAV_FLAG_GPS_FILTER;
  if (GPS_LOW_SPEED_D_FILTER) nav_flags += I2C_NAV_FLAG_LOW_SPEED_D_FILTER; 
  
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_NAV_FLAGS);
    i2c_write(nav_flags);
    
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_WP_RADIUS);
    i2c_write(GPS_WP_RADIUS & 0x00FF); // lower eight bit   
    i2c_write(GPS_WP_RADIUS >> 8); // upper eight bit
#endif
}

#if defined (TINY_GPS)
int32_t GPS_coord_to_decimal(struct coord *c) {
	#define GPS_SCALE_FACTOR 10000000L
	uint32_t deg = 0;
	deg = (uint32_t)c->deg * GPS_SCALE_FACTOR;

	uint32_t min = 0;
	min = (uint32_t)c->min * GPS_SCALE_FACTOR;
	/* add up the BCD fractions */
	uint16_t divisor = (uint16_t)GPS_SCALE_FACTOR/10;
	for (uint8_t i=0; i<NMEA_MINUTE_FRACTS; i++) {
		uint8_t b = c->frac[i/2];
		uint8_t n = (i%2 ? b&0x0F : b>>4);
		min += n*(divisor);
		divisor /= 10;
	}

	/* now sum up degrees and minutes */
	return deg + min/60;
}
#endif

//OK here is the onboard GPS code
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD) || defined(TINY_GPS)

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles	
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat, int32_t* lon) {
  GPS_WP[LAT] = *lat;
  GPS_WP[LON] = *lon;
 
  GPS_calc_longitude_scaling(*lat);
  GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);

  nav_bearing = target_bearing;
  GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
  original_target_bearing = target_bearing;
  waypoint_speed_gov = NAV_SPEED_MIN;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp() {
  int32_t temp;
  temp = target_bearing - original_target_bearing;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);	// we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree 
//
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
  uint32_t d1;
  int32_t  d2;
  GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
  *dist = d1 / 100;          //convert to meters
  *bearing = d2 /  100;      //convert to degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity(){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;
  // y_GPS_speed positve = Up
  // x_GPS_speed positve = Right

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;
  
    actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
    actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;
  
    speed_old[_X] = actual_speed[_X];
    speed_old[_Y] = actual_speed[_Y];
  }
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//	100 	= 1m
//	1000 	= 11m	 = 36 feet
//	1800 	= 19.80m = 60 feet
//	3000 	= 33m
//	10000 	= 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold() {
  int32_t p,i,d;						
  int32_t output;
  int32_t target_speed;
  
  for (uint8_t axis=0;axis<2;axis++) {
    target_speed = pi_poshold[axis].get_p(error[axis],&posholdPID);			// calculate desired speed from lon error
    rate_error[axis] = target_speed - actual_speed[axis];	                // calc the speed error
  
    p = pid_poshold_rate[axis].get_p(rate_error[axis],&poshold_ratePID);
    i = pid_poshold_rate[axis].get_i(rate_error[axis] + error[axis], &dTnav,&poshold_ratePID);
    d = pid_poshold_rate[axis].get_d(error[axis], &dTnav,&poshold_ratePID);
    d = constrain(d, -2000, 2000);
    // get rid of noise
    #if defined(GPS_LOW_SPEED_D_FILTER)
      if(abs(actual_speed[axis]) < 50) d = 0;
    #endif
    output  = p + i + d;

    nav[axis]  = constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX); 		
    pid_nav[axis].set_integrator(pid_poshold_rate[axis].get_integrator());
  }
}
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(int max_speed) {
  float trig[2];
  // push us towards the original track
  GPS_update_crosstrack();

  // nav_bearing includes crosstrack
  float temp = (9000l - nav_bearing) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  for (uint8_t axis=0;axis<2;axis++) {
    rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis]; 
    rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
    // P + I + D
    nav[axis]      =
        pid_nav[axis].get_p(rate_error[axis],&navPID)
       +pid_nav[axis].get_i(rate_error[axis], &dTnav,&navPID)
       +pid_nav[axis].get_d(rate_error[axis], &dTnav,&navPID);
    nav[axis]      = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    pid_poshold_rate[axis].set_integrator(pid_nav[axis].get_integrator());
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line 
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void) {
  if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
    float temp = (target_bearing - original_target_bearing) * RADX100;
    crosstrack_error = sin(temp) * (wp_distance * CROSSTRACK_GAIN);	 // Meters we are off track line
    nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
    nav_bearing = wrap_36000(nav_bearing);
  }else{
    nav_bearing = target_bearing;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow 
// speed rampup when starting a navigation
//
//	|< WP Radius
//	0  1   2   3   4   5   6   7   8m
//	...|...|...|...|...|...|...|...|
//		  100  |  200	  300	  400cm/s
//	           |  		 		            +|+
//	           |< we should slow to 1.5 m/s as we hit the target
//
static int16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  // max_speed is default 400 or 4m/s
  if(_slow){
    max_speed = min(max_speed, wp_distance / 2);
    max_speed = max(max_speed, 0);
  }else{
    max_speed = min(max_speed, wp_distance);
    max_speed = max(max_speed, NAV_SPEED_MIN);	// go at least 100cm/s
  }

  // limit the ramp up of the speed
  // waypoint_speed_gov is reset to 0 at each new WP command
  if(max_speed > waypoint_speed_gov){
    waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms
    max_speed = waypoint_speed_gov;
  }
  return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}

int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
  return ang;
}

// This code is used for parsing NMEA data
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD)

/* Alex optimization 
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
  resolution also increased precision of nav calculations
*/
uint32_t GPS_coord_to_degrees(char* s) {
  char *p = s, *d = s;
  uint8_t min, deg = 0;
  uint16_t frac = 0, mult = 10000;
  
  while(*p) {                                   // parse the string until its end
    if (d != s) {frac+=(*p-'0')*mult;mult/=10;} // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
    if (*p == '.') d=p;                         // locate '.' char in the string
    p++;
  }
  if (p==s) return 0;
  while (s<d-2) {deg *= 10;deg += *(s++)-'0';}  // convert degrees : all chars before minutes ; for the first iteration, deg = 0
  min = *(d-1)-'0' + (*(d-2)-'0')*10;           // convert minutes : 2 previous char before '.'
  return deg * 10000000UL + (min * 100000UL + frac)*10UL / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;
  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9')	tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {		// convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
      else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
      else if (param == 6)                     {flag.GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}	// altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*514444L)/100000L;}	// speed in cm/s added by Mis
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  if (frame) GPS_Present = 1;
  return frameOK && (frame==FRAME_GGA);
}
#endif //SERIAL GPS
#endif //ONBOARD GPS CALC

#endif // GPS
