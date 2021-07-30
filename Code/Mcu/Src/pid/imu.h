
// Defines
#define SDA 26
#define SCL 27

// Global function prototypes
void imu_setup(void);
void imu_setup_dmp(void); 
void getEulerAngles_dmp(float *angles);
//double getTilt(void);
void getAccelGyro(float* ay, float* az, float* gx);
