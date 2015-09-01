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

// Sonar Filtering Variables
uint16_t snr_distcm_k1 = 0;
uint16_t snr_distcm_k2 = 0;
int16_t snr_distcm = 0;
uint16_t raw_sn = 0;
uint16_t rw_snr_k1 = 0;
uint16_t rw_snr_k2 = 0;

// Pixy Filtering Variables
Vector3f px_err_k2;
Vector3f px_err_k1;
Vector3f px_err_fil;
uint16_t zrs_chck = 0;

#endif  // USERHOOK_VARIABLES


