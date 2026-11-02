#include <math.h>
#include "main.h"
#include "Global.h"
#include "IMU.h"
#include "Kalman.h"
#include "ICM-42670.h"

int16_t accel_offsetX=0,accel_offsetY=0;
extern float gyro[],accel[];
extern uint32_t ClkThisRound;
 float Z_GyroRate,accel_offsetZ=0;// Radian
extern float TempMPU;
extern uint8_t MountSensorType ,Rx_type,Flag10Hz;   


void LogWrite(uint32_t Round);

Kalman_f KalmanGyroX = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
Kalman_f KalmanGyroY = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanGyroZ = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanGyroX_Acc = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
Kalman_f KalmanGyroY_Acc = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanGyroZ_Acc = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};


Kalman_f KalmanAccelX = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
Kalman_f KalmanAccelY = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanAccelZ = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
void HeadSensorConvert (uint8_t Heading,float *Gyro,float *Accel){
float GyroTemp[3],AccelTemp[3];

GyroTemp[0]=Gyro[0];
GyroTemp[1]=Gyro[1];
GyroTemp[2]=Gyro[2];

AccelTemp[0]=Accel[0];
AccelTemp[1]=Accel[1];
AccelTemp[2]=Accel[2];

 if(Heading==2){
   Gyro[0]=0-Gyro[0];
   Gyro[1]=0-Gyro[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=0-AccelTemp[1];
  }

else if(Heading==3){
   Gyro[1]=0-Gyro[1];
   Gyro[2]=0-Gyro[2];
   
   
   Accel[1]=0-AccelTemp[1];
   Accel[2]=0-AccelTemp[2];

  }
  else if(Heading==4){
   Gyro[0]=0-GyroTemp[0];
   Gyro[2]=0-GyroTemp[2];
   
   Accel[0]=0-AccelTemp[0];
   Accel[2]=0-AccelTemp[2];
  }
 else if(Heading==5){
   Gyro[1]=0-GyroTemp[2];
   Gyro[2]=GyroTemp[1];
   
   Accel[1]=0-AccelTemp[2];
   Accel[2]=AccelTemp[1];
  }
  else if(Heading==6){
   Gyro[0]=0-GyroTemp[0];
   Gyro[1]=0-GyroTemp[2];
   Gyro[2]=0-GyroTemp[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=0-AccelTemp[2];
   Accel[2]=0-AccelTemp[1];
   
  }
 else if(Heading==7){
   Gyro[1]=GyroTemp[2];
   Gyro[2]=0-GyroTemp[1];
   
   Accel[1]=AccelTemp[2];
   Accel[2]=0-AccelTemp[1];

  }
  else if(Heading==8){
   Gyro[0]=0-GyroTemp[0];
   Gyro[1]=GyroTemp[2];
   Gyro[2]=GyroTemp[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=AccelTemp[2];
   Accel[2]=AccelTemp[1];
  }
}





 //uint8_t temUsartTx[64];  
__IO uint32_t CountStable=0;
__IO float TestFloat;


