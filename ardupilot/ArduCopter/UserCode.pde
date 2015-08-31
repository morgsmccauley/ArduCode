/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define XCENTRE 154.5f
#define YCENTRE 101
#define MXPIXCONV 0.0043f
#define CXPIXCONV 0.00f
#define MYPIXCONV 0.0042f
#define CYPIXCONV 0.00f

#define TAU 1.0f
#define TK 0.1f
#define THRES 10
#define FILTERDIFF 10

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

float filter_pixy_data(float raw, float filt_k1, float filt_k2, uint16_t upper)
{
	float filt = -1;
    if (raw == 0 && ((filt_k1 - raw) > THRES || (filt_k1 - raw) < -THRES))
	{
        filt = filt_k1 + (filt_k1 - filt_k2); // Check this does not result in too great a rate of increase
	}
	// Check if calculated value is outside of reasonable bounds
    if ((filt < 0) || (filt > upper))
	{
		filt = raw;
	}
	
    return filt;

}

void update_pixy_data()
{
	// Update Previous Filter Values
    px_err_k2 = px_err_k1;
    px_err_k1 = px_err_fil;	
	// Read New Data
	Vector3f raw_pixy_error = update_irlock(1);
	// zrs_chck is the number of times uninterrupted the pixy hasn't been able to update
	zrs_chck = (zrs_chck + raw_pixy_error.z) * raw_pixy_error.z;	// 'z' is being used as a flag when no update has been made
	
	// Write Pin for LED
    hal.rcout->write(4, 0);
	
	// Filter X and Y readings

	// Check the number of iterations since last valid update
	if (zrs_chck < 8)
	{
		// Update using previous errors
		px_err_fil.x = filter_pixy_data(raw_pixy_error.x, px_err_k1.x, px_err_k2.x, 320);
		px_err_fil.y = filter_pixy_data(raw_pixy_error.y, px_err_k1.y, px_err_k2.y, 240);
	} 
	else
	{
		// Update using an arbitrary fixed rate of -5 (set by FILTERDIFF)
		px_err_fil.x = filter_pixy_data(raw_pixy_error.x, px_err_k1.x, (px_err_k1.x + FILTERDIFF), 320);
		px_err_fil.y = filter_pixy_data(raw_pixy_error.y, px_err_k1.y, (px_err_k1.y + FILTERDIFF), 240);
	}

    if(AP_Notify::flags.armed)
    {
        Log_Write_Airspeed(raw_pixy_error.x, raw_pixy_error.y, px_err_fil.x, px_err_fil.y);
    }
    
	if (!(px_err_fil.x == 0 && px_err_fil.y == 0))
    {
		// Convert pixels to cms  
        pixy_error.x = model_X(px_err_fil.x, sonar_distcm);
        pixy_error.y = model_Y(px_err_fil.y, sonar_distcm);

        hal.console->printf_P(PSTR("YO\n"));
    }
    else {

        pixy_error.x = 0.0f;
        pixy_error.y = 0.0f;
    }  

    hal.console->printf_P(PSTR("PIXY: %f, %f \n"), pixy_error.x, pixy_error.y);
}

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //airspeed.init();
    //airspeed.calibrate(false);

    hal.rcout->enable_ch(5);
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here

    Vector2f flow = optflow.flowRate();
    Vector2f body = optflow.bodyRate();

    opt_vel.x = (flow.x - body.x) * sonar_distcm;
    opt_vel.y = (flow.y - body.y) * sonar_distcm;
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
	update_pixy_data();
    
    //airspeed.read();

    //float temp = 0;
    //airspeed.get_temperature(temp);

    //hal.console->printf_P(PSTR("airspeed: %f"), airspeed.get_airspeed());
    //hal.console->printf_P(PSTR("x: %f, y: %f\n"), pixy_error.x, pixy_error.y);
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here

	//Log_Write_Sonar((sonar.distance_cm() / 100.0f), sonar.voltage_mv());

	// Read, Filter, & Log Sonar Data
	sonar_distcm_prvB = sonar_distcm_prvA;
	sonar_distcm_prvA = sonar_distcm_A;
	
	sonar_distcm_prv = sonar_distcm;
	
	// Low Pass Filter
	sonar_distcm = (TAU * sonar_distcm_prv + TK * sonar.distance_cm()) / (TAU + TK);
	
	// Outlier Filter
	if((sonar.distance_cm() > sonar_distance_prv + THRES) || (sonar.distance_cm() < sonar_distance_prv - THRES))
	{
        sonar_distcm_A = sonar_distcm_prvA + (sonar_distcm_prvA - sonar_distcm_prvB)/2;
	}
    else{
        sonar_distcm_A = sonar.distance_cm();
	}

    if(AP_Notify::flags.armed){
        Log_Write_Sonar(sonar_distcm_A, sonar.distance_cm(), sonar.voltage_mv());
        hal.console->printf_P(PSTR("logging sonar\n"));
    }
	
	sonar_distance_prv = sonar.distance_cm();
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