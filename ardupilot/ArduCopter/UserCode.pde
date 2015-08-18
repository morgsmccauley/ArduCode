/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define XCENTRE 154.5f
#define YCENTRE 101
#define MXPIXCONV 0.0043f
#define CXPIXCONV 0.00f
#define MYPIXCONV 0.0042f
#define CYPIXCONV 0.00f
#define INTERNALKP 5.0f

#define TAU 1.0f
#define TK 0.1f

float model_X(int raw_x, int curr_alt)
{
    // Calculate corresponding Angle from centre of lens:
    float obj_angle = MXPIXCONV * (raw_x - XCENTRE) + CXPIXCONV;
    // Calculate lateral distance using depth and angle:
    return (curr_alt * tan(obj_angle));
}

float model_Y(int raw_y, int curr_alt)
{
    // Calculate corresponding Angle from centre of lens:
    float obj_angle = MYPIXCONV * (raw_y - YCENTRE) - CYPIXCONV;
    // Calculate lateral distance using depth and angle:
	return (curr_alt * tan(obj_angle));
}

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //airspeed.init();
    //airspeed.calibrate(false);
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
    Vector2f raw_pixy_error = update_irlock(1);

    //convert pixels to cms

    //hal.console->printf_P(PSTR("sonar: %d\n"), sonar.distance_cm());

    if (!(raw_pixy_error.x == 0 && raw_pixy_error.y == 0))
    {
        pixy_error.x = model_X(raw_pixy_error.x, sonar_distcm);
        pixy_error.y = model_Y(raw_pixy_error.y, sonar_distcm);   
    }
    else
    {
        pixy_error.x = 0;
        pixy_error.y = 0;
    }


    //airspeed.read();

    float temp = 0;
    //airspeed.get_temperature(temp);

    //Log_Write_Airspeed(airspeed.get_airspeed(), airspeed.get_raw_airspeed(), airspeed.get_airspeed_ratio(), temp);

    //hal.console->printf_P(PSTR("airspeed: %f"), airspeed.get_airspeed());
    //hal.console->printf_P(PSTR("x: %f, y: %f\n"), pixy_error.x, pixy_error.y);
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
	
	// Read, Filter, & Log Sonar Data
	sonar_distcm_prv = sonar_distcm;
	sonar_distcm = (TAU * sonar_distcm_prv + TK * sonar.distance_cm()) / (TAU + TK);
	Log_Write_Sonar(sonar_distcm, sonar.distance_cm(), sonar.voltage_mv());
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif