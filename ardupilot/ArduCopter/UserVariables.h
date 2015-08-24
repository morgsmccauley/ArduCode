/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

Vector2f pixy_error;

uint16_t sonar_distcm = 0;
uint16_t sonar_distcm_prv = 0;
uint16_t sonar_distcm_prvB = 0;
uint16_t sonar_distcm_prvA = 0;
uint16_t sonar_distance_prv = 0;
uint16_t sonar_distcm_A = 0;

// Pixy Filtering Variables
Vector2f rw_px_err_prv;
Vector2f rw_px_err_fil;

#endif  // USERHOOK_VARIABLES


