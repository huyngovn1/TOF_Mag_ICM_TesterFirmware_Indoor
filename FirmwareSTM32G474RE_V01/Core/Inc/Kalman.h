#include <stdint.h>
// Kalman structure
#define  KALMAN_Q 0.001f   // Tang keo nhanh
#define  KALMAN_BIAS 0.003f
#define  KALMAN_R 0.06f   // Tang keo cham, min hon

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;
    
typedef struct {
    float Q;
    float Q_bias;
    float R_measure;
    float Out;
    float bias;
    float P[2][2];
} Kalman_f;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
float  KalmanFiller(Kalman_f *Kalman, float newAngle,  float dt);