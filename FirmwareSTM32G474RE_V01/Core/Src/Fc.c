/*
 * Fc.c
 *
 *  Created on: Jun 24, 2026
 *      Author: HP
 */
#include "Global.h"
#include "Kalman.h"
#include "ICM-42670.h"
#include "IMU.h"
#include "Fc.h"
#include "Rx.h"
#include "MyAHRS.h"
extern Imu_t Imu;
Fc_t Fc;
FcConfig_t FcConfig= {
		.ImuOrient=1,
		.ThroIdling=6,
		.RcDeadBand=16,
		.ThroMin=1100*4,
		.ThroMax=1940*4,
		.GainRoll=50,
		.GainRollP=50,
		.GainRollI=50,
		.GainRollD=50,
		.GainPitch=50,
		.GainPitchP=50,
		.GainPitchI=50,
		.GainPitchD=50,
		.GainYaw=50,
		.GainYawP=50,
		.GainYawI=50,
		.GainYawD=50,
		.RollRate=50,
		.PitchRate=50,
		.YawRate=50,
		.GpsPos=10,
		.FcPosY=5,
		.FcPosZ=2
};
Kalman_f KalmanXCON = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};


Kalman_f KalmanYCON = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanXCON_D = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanYCON_D = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};


uint16_t Motor[10]={6080},IDLE_THRO=168;
extern float PercomSpeed,accel_offsetZ;
extern uint16_t RXVal[],RXCen[];
extern float AxGPS_Body,AyGPS_Body;
extern int16_t MagRaw[],MagOffset[],MagTravel[];
extern Rx_t Rx;
extern float X_CON_GPS,Y_CON_GPS,VGPS,cosangle0,cosangle1;
__IO float Angle[3],I_AngleFlybar[3],GyroLP[3];
uint8_t CyclicD_Gain=50,CyclicP_Gain=50,CyclicI_Gain=50,RcDeadBand=16,GainRoll=50,GainPitch=50;
uint8_t GPSHomeSet,Armed;
uint8_t RudP_Gain=50,RudI_Gain=50,RudD_Gain=50,GainRud=50,YawRate=100;
float GyroAcc[3],AngleFlybar[3];
__IO uint32_t TxIdleTime;
int16_t X_CON,Y_CON,Z_CON;
extern int16_t GyroAvg[3], AcellAvg[3];
__IO float gyroOffsetFist[3];
float accelNonOfset[3];
extern float AutoPitch;

void Servo1Out(uint16_t value)	//Ail CH
{
	TIM1->CCR4 = value; //2100us
}
void Servo2Out(uint16_t value)	//Ail CH
{
	TIM1->CCR3 = value; //2100us
}
void Servo3Out(uint16_t value)	//Ail CH
{
	TIM1->CCR1 = value; //2100us
}
void Servo4Out(uint16_t value)	//Ail CH
{
	TIM1->CCR2 = value; //2100us
}

void quadXMixerX4(uint16_t throttle_con ,int16_t x_con ,int16_t y_con ,int16_t r_con )
    {

	        Motor[0] = throttle_con - y_con +x_con + 1*r_con ;
	        Motor[1] = throttle_con + y_con +x_con - 1*r_con ;

	        Motor[2] = throttle_con + y_con -x_con + 1*r_con ;
	        Motor[3] = throttle_con - y_con -x_con - 1*r_con ;

    }



uint8_t Mixer(int16_t x_con ,int16_t y_con ,int16_t r_con,int16_t ThroCon )//Config 3
{      uint8_t ReVal=0,i;
         //if(ThroCon>(THRO_MAX-640)) ThroCon=(THRO_MAX-640);
         //else if(ThroCon<(THRO_MIN+IDLE_THRO)) ThroCon=(THRO_MIN+IDLE_THRO);
      if(Armed==2) {
		 if(r_con>1000) r_con=1000;
		 else if(r_con<(-1000)) r_con=-1000;

		 quadXMixerX4( ThroCon,x_con,y_con,r_con);
			 // Uu tien Cyclic, tu ha Pitch
		 uint16_t MaxMotor=0,MinMotor=10000;
		 for (i=0;i<4;i++)if(Motor[i]<MinMotor)MinMotor=Motor[i];
		 if(MinMotor<(THRO_MIN+IDLE_THRO)) {
			 for (i=0;i<4;i++)Motor[i]+=(THRO_MIN+IDLE_THRO)-MinMotor;
			 ReVal=1;
		 }

		 for (i=0;i<4;i++)if(Motor[i]>MaxMotor)MaxMotor=Motor[i];
		 if(MaxMotor>THRO_MAX) {
			 for (i=0;i<4;i++)Motor[i]-=MaxMotor-THRO_MAX;
			 ReVal=2;
		 }
      }
      else 	 {
    	  for (i=0;i<4;i++)Motor[i]=THRO_MIN;
    	  AngleFlybar[0]-=AngleFlybar[0]*0.001f;
    	  AngleFlybar[1]-=AngleFlybar[1]*0.001f;
    	  AngleFlybar[2]-=AngleFlybar[2]*0.001f;

    	  I_AngleFlybar[0]-=I_AngleFlybar[0]*0.001f;
    	  I_AngleFlybar[1]-=I_AngleFlybar[1]*0.001f;
    	  I_AngleFlybar[2]-=I_AngleFlybar[2]*0.001f;
      }

  	 Servo1Out(Motor[0]);
     Servo2Out(Motor[1]);
	 Servo3Out(Motor[2]);
	 Servo4Out(Motor[3]);

     return ReVal;
}

int16_t Roll_SetAngle(float AngleSet,float AngleIn){
    static float AngleSetLast=0,AngleSetLastXin=0;
    if(AngleSet>(AngleSetLastXin+1.0f))AngleSet=(AngleSetLastXin+1.0f);
    else if(AngleSet<(AngleSetLastXin-1.0f))AngleSet=(AngleSetLastXin-1.0f);
    AngleSetLastXin=AngleSet;

    KalmanFiller(&KalmanXCON,AngleSet, 25.0f/(float)Frq_Update);  //5Hz LP
    float Err=(KalmanXCON.Out-AngleIn);
	float PDelay=Err*(float)CyclicP_Gain*0.3f;
//	PXDelay+=(P-PXDelay)*0.06f; //5Hz

/*	if(Armed>=2)I_AngleFlybar[0]+=Err*(float)CyclicI_Gain*0.001f; //&&TakeOffOK==0
	else I_AngleFlybar[0]-=I_AngleFlybar[0]*0.002f;
	if(I_AngleFlybar[0]>400)I_AngleFlybar[0]=400;
	else if(I_AngleFlybar[0]<-400)I_AngleFlybar[0]=-400;*/

	float Dcon=KalmanFiller(&KalmanXCON_D,KalmanXCON.Out-AngleSetLast, 25.0f/(float)Frq_Update);  //5Hz LP

	float D=(Dcon*((float)Frq_Update*0.9f)- GyroLP[0]- (GyroAcc[0]*8.0f))*(float)CyclicD_Gain/16.0f;
	float OutPut=((PDelay+I_AngleFlybar[0]+D)*(float)GainRoll)/100.0f; //+(VTx[0]/Rx_ms)

	AngleSetLast=KalmanXCON.Out;
	//if(OutPut>1600)OutPut=1600; else if(OutPut<-1600)OutPut=-1600;
    return (int16_t)OutPut;
}

int16_t Pitch_SetAngle(float AngleSet,float AngleIn){
   static float AngleSetLast=0,AngleSetLastYin=0;
   if(AngleSet>(AngleSetLastYin+1.0f))AngleSet=(AngleSetLastYin+1.0f);
   else if(AngleSet<(AngleSetLastYin-1.0f))AngleSet=(AngleSetLastYin-1.0f);
   AngleSetLastYin=AngleSet;

   KalmanFiller(&KalmanYCON,AngleSet, 25.0f/(float)Frq_Update);  //5Hz LP
   float Err=(KalmanYCON.Out-AngleIn);
			float PYDelay=Err*(float)CyclicP_Gain*0.3f;
//			PYDelay+=(P-PYDelay)*0.06f;

/*			if(Armed>=2)I_AngleFlybar[1]+=Err*(float)CyclicI_Gain*0.001f;
			else I_AngleFlybar[1]-=I_AngleFlybar[1]*0.002f;
			if(I_AngleFlybar[1]>400)I_AngleFlybar[1]=400;
			else if(I_AngleFlybar[1]<-400)I_AngleFlybar[1]=-400;*/

			float Dcon=KalmanFiller(&KalmanYCON_D,KalmanYCON.Out-AngleSetLast, 25.0f/(float)Frq_Update);  // LP

			float D=(Dcon*((float)Frq_Update*0.9f)- GyroLP[1]- (GyroAcc[1]*8.0f))*(float)CyclicD_Gain/16.0f;
			float OutPut=((PYDelay+I_AngleFlybar[1]+D)*(float)GainPitch)/100.0f; //+(VTx[0]/Rx_ms)

			AngleSetLast=KalmanYCON.Out;
			//if(OutPut>1600)OutPut=1600; else if(OutPut<-1600)OutPut=-1600;
   return (int16_t)OutPut;
}

int16_t PID_DroneYaw(void){
   float ReturnVal;
   //static float PZDelay=0;
        Rx.TxMoveDelay[2]+=((float)Rx.TxMove2[2]-Rx.TxMoveDelay[2])*0.008f; // Delay
        if(Rx.TxMoveDelay[2]>(0.15f+(float)Rx.TxMove2[2]))Rx.TxMoveDelay[2]-=0.15f;
        else if(Rx.TxMoveDelay[2]<((float)Rx.TxMove2[2]-0.15f))Rx.TxMoveDelay[2]+=0.15f;
        else Rx.TxMoveDelay[2]=(float)Rx.TxMove2[2];

     	AngleFlybar[2]+=(float)(Rx.TxMoveDelay[2]*YawRate)/((float)Frq_Update*1680.0f);
        AngleFlybar[2]-=GyroLP[2]/(float)Frq_Update;

         //--------------End I---------------
        if(AngleFlybar[2]>25.0f)AngleFlybar[2]=25.0f; else if(AngleFlybar[2]<(-25.0f))AngleFlybar[2]=(-25.0f);

       // AngleFlybarDelay[2] = KalmanFiller(&KalmanTailP,AngleFlybar[2], 1.0f/(float)Frq_Update);  //1Hz LP
       // PZDelay+=(AngleFlybar[2]-PZDelay)*0.06f; // Delay 5Hz

    	ReturnVal=(AngleFlybar[2]*0.6f)*(float)RudP_Gain + (((Rx.TxMoveDelay[2]*(float)YawRate)/1680.0f-(GyroAcc[2]*5.0f +GyroLP[2]))*(float)RudD_Gain/(2.5f));
        ReturnVal=(ReturnVal*(float)GainRud)/100.0f ;
	return (int16_t)ReturnVal;
}


void FlightControlLoop(void){

}

void FlightControlLoop2(void)
{

}


