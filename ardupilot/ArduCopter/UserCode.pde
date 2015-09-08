/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define XCENTRE 154.5f
#define YCENTRE 101
#define MXPIXCONV 0.0043f
#define CXPIXCONV 0.00f
#define MYPIXCONV 0.0042f
#define CYPIXCONV 0.00f

#define TAU 1.0f
#define TK 0.1f
#define THRESPX 10
#define THRESSN 10
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
    if (raw == 0 && ((filt_k1 - raw) > THRESPX || (filt_k1 - raw) < -THRESPX))
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

void update_sonar_data()
{
	// Read, Filter, & Log Sonar Data
	// Update previous filtered values
	snr_distcm_k2 = snr_distcm_k1;
	snr_distcm_k1 = snr_distcm;
	// Set Temp value for current cm reading - Ensures update if filtering not necessary
	snr_distcm = -1;
	// Read current raw value & update previous raw values
	rw_snr_k2 = rw_snr_k1;
	rw_snr_k1 = raw_sn;
	raw_sn = sonar.distance_cm();

	// Outlier Filter
	if(abs(raw_sn - rw_snr_k1) > THRESSN || abs(rw_snr_k1 - rw_snr_k2) > THRESSN)
	{
		int16_t diff = (snr_distcm_k1 - snr_distcm_k2);
        if (abs(diff) > THRESSN)
		{
			diff = (diff / abs(diff)) * THRESSN;
		}
        snr_distcm = snr_distcm_k1 + diff;
	}
	// Sanity Check on filtered value
    if ((snr_distcm < 0) || (snr_distcm > 650)) 
	{
        snr_distcm = raw_sn;
	}
	// Log Filtered & Raw Data
    if(AP_Notify::flags.armed){
        Log_Write_Sonar(snr_distcm, raw_sn, sonar.voltage_mv());
    }

    //hal.console->printf_P(PSTR("SONAR: %d, %d\n"), snr_distcm, sonar.voltage_mv());

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
    
	if (!(px_err_fil.x == 0 && px_err_fil.y == 0))
    {
		// Convert pixels to cms
        pixy_error.x = model_X(px_err_fil.x, snr_distcm);
        pixy_error.y = model_Y(px_err_fil.y, snr_distcm);   
    }
    else {

        pixy_error.x = 0.0f;
        pixy_error.y = 0.0f;
    }

    
    if(AP_Notify::flags.armed)
    {
        Log_Write_Airspeed(pixy_error.x, pixy_error.y, px_err_fil.x, px_err_fil.y);
    }

    //hal.console->printf_P(PSTR("PIXY: %f, %f \n"), pixy_error.x, pixy_error.y);
}

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //airspeed.init();
    //airspeed.calibrate(false);

    //hal.rcout->enable_ch(5);

    low_pass_filter_x.set_cutoff_frequency(0.3185f);
    low_pass_filter_x.reset(0);

    low_pass_filter_y.set_cutoff_frequency(0.3185f);
    low_pass_filter_y.reset(0);
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
/*
    Vector2f flow = optflow.flowRate();
    Vector2f body = optflow.bodyRate();
    Vector2f temp;

    temp.x = (flow.x - body.x) * snr_distcm;
    temp.y = (flow.y - body.y) * snr_distcm;


    float filtered_value_x = low_pass_filter_x.apply(temp.x, 0.01f);
    float filtered_value_y = low_pass_filter_y.apply(temp.y, 0.01f);

    opt_vel.x = filtered_value_y;
    opt_vel.y = -filtered_value_x;
*/
    
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
	update_sonar_data();
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