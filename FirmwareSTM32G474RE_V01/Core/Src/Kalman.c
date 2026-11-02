#include <math.h>
#include "Kalman.h"
//https://github.com/leech001/MPU6050
#define RAD_TO_DEG 57.295779513082320876798154814105
uint32_t timer;
extern float GyroLP[],accelLP[];
 Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};


void KalmanAHRS_Update(float *GyroLP, float *accelLP,float *Angle) {

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
   /*
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    accelLP[0] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    accelLP[1] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    accelLP[2] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    GyroLP[1]ro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    GyroLP[1]ro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    GyroLP[1]ro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = accelLP[0] / 16384.0;
    DataStruct->Ay = accelLP[1] / 16384.0;
    DataStruct->Az = accelLP[2] / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    GyroLP[0] = GyroLP[1]ro_X_RAW / 131.0;
    GyroLP[1] = GyroLP[1]ro_Y_RAW / 131.0;
    GyroLP[2] = GyroLP[1]ro_Z_RAW / 131.0;
   GyroLP[0],GyroLP[1],GyroLP[2],accelLP[0],accelLP[1],accelLP[2],Angle
*/
    // Kalman angle solve
  double dt =1.5;//  double dt = (double) (HAL_GetTick() - timer) / 1000;
  //  timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(accelLP[0] * accelLP[0] + accelLP[2] * accelLP[2]);
    if (roll_sqrt != 0.0) {
        roll = atan(accelLP[1] / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-accelLP[0], accelLP[2]) * RAD_TO_DEG;
    if ((pitch < -90 && Angle[1] > 90) || (pitch > 90 && Angle[1] < -90)) {
        KalmanY.angle = pitch;
        Angle[1] = pitch;
    } else {
        Angle[1] = Kalman_getAngle(&KalmanY, pitch, GyroLP[1], dt);
    }
    if (fabs(Angle[1]) > 90)
        GyroLP[0] = -GyroLP[0];
    Angle[0] = Kalman_getAngle(&KalmanX, roll, GyroLP[1], dt);

}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

float  KalmanFiller(Kalman_f *Kalman, float Input,float dt) {

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = Input - Kalman->Out;
    Kalman->Out += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->Out;
};
