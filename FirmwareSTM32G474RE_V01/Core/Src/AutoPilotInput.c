/*
 * AutoPilotInput.c
 *
 *  Created on: Jul 6, 2026
 *      Author: HP
 */
#include "main.h"
#include <math.h>
#include "Global.h"
#include "IMU.h"
#include "Fc.h"
#include "AutoPilotInput.h"
#include "AutoPilot.h"
extern Rx_t Rx;
extern uint8_t MagCalib,MagCalibLast,FlyingStatus,GPSHomeWaiting;
extern int16_t MagRaw[],MagOffset[];
extern float GyroLP[],Angle[];
uint8_t AccCalib=1,MagAutoCalibFly=1;
int16_t MagTravel[3]={1200,1200,1200};
uint32_t clk_MagCalib=0;
uint8_t MagCalib=0,MagCalibLast;
int16_t XMagMax,XMagMin,YMagMax,YMagMin;
extern float angle_HMC;
float sinHMC,cosHMC;
extern float HoverAngle[],AngleFlybar[];
extern MTF_t MTF;
__IO float MovingSpeedToF[3];

void CompassCab(int16_t * MagOffset ){ /// Rotation 2 round for Done
     static float AngleZRotation=0;
	 if(MagCalib)
	 {
	  AngleZRotation+=GyroLP[2];
	  if( MagRaw[0]<XMagMin){
	    XMagMin=MagRaw[0];
	  }
	  else if( MagRaw[0]>XMagMax){
	     XMagMax= MagRaw[0];
	   }

	  if( MagRaw[1]<YMagMin){
	     YMagMin=MagRaw[1];
	   }
	  else if(MagRaw[1]>YMagMax){
	    YMagMax=MagRaw[1];
	  }
	  clk_MagCalib++;
          if(Angle[0]>(ONE_DEG*30) || Angle[0]<(ONE_DEG*-30)|| Angle[1]>(ONE_DEG*30) || Angle[1]<(ONE_DEG*-30)){ // Fail
                           AngleZRotation=0;
                           XMagMax=XMagMin= MagRaw[0];
                           YMagMax=YMagMin=MagRaw[1];
          }

	  if((AngleZRotation>(Frq_Update*720))|| (AngleZRotation<(Frq_Update*(-720)))){ // Xoay du 2 vong trong 60s
			MagOffset[0]=(XMagMax+XMagMin)/2;
			MagOffset[1]=(YMagMax+YMagMin)/2;

                        MagTravel[0]=(XMagMax-XMagMin)/2;
                        MagTravel[1]=(YMagMax-YMagMin)/2;

		if(MagTravel[0]>400 && MagTravel[1]>400&& MagOffset[0]<4000&& MagOffset[0]>-4000&& MagOffset[1]<4000 && MagOffset[1]>-4000 ){
                        MagCalib=0;
//			Write_Flash_Config(AdrFlashData,((MagOffset[1]<<16)|(MagOffset[0])));
//                        Write_Flash_Config(AdrFlashData+8,(MagTravel[0]|((int32_t)MagTravel[1]<<16)));
	        } else {
                           XMagMax=XMagMin= MagRaw[0];
                           YMagMax=YMagMin=MagRaw[1];
                           AngleZRotation=0;
                }
	  }
	 }
	 else {
             clk_MagCalib=0;
             XMagMax=XMagMin= MagRaw[0];
             YMagMax=YMagMin=MagRaw[1];
             AngleZRotation=0;
         }

         if(MagCalibLast &&MagCalib==0) { if(GPSHomeWaiting) LED1_LO;  else LED1_HI;  }
         MagCalibLast=MagCalib;
}
void CompassCabInLoop(float GyroZ, int16_t *Mag){ //50Hz

}

void MagUpdateAngle(int16_t* MagRaw){

}

__IO uint8_t MtfPosLocked=0,TofXLocked=0,TofYLocked;
extern float VX_Accel,VY_Accel;
uint8_t XyPosition3Sensors(int32_t mmX,int32_t mmY, int32_t *mmXset, int32_t *mmYset){ //mm input
	static int32_t mmXlast=0, mmYlast=0;
	static int32_t OffetX=0,OffetY=0;
	int32_t dx=mmX-mmXlast;
	int32_t dy=mmY-mmYlast;

    if(dx>100||dx<-100){

    }


	mmXlast=mmX;
	mmYlast=mmY;
}

uint8_t XyPosition3Sensors2(uint32_t *mmIn,float *VxAccel, float *VyAccel){ //mm input
	uint8_t ReVal=0;
	static uint32_t mmInLast[3]={0,0,0};

	MtfPosLocked=0;
	TofXLocked=0;
	TofYLocked=0;

	float DriftX,DriftY; //cm/s
	 uint32_t TickNow=HAL_GetTick();

	if(MTF.flow_quality){
		DriftX=MTF.moved_x;
		if(DriftX>(*VxAccel+50))DriftX=*VxAccel+50;
		else if(DriftX<(*VxAccel-50))DriftX=*VxAccel-50;
		else MtfPosLocked=1;
		*VxAccel+=(DriftX-*VxAccel)/(2*Frq_Update);

		 DriftY=MTF.moved_y;
		 if(DriftY>(*VyAccel+50))DriftY=*VyAccel+50;
		 else if(DriftY<(*VyAccel-50))DriftY=*VyAccel-50;
		 else MtfPosLocked=1;
		 *VyAccel+=(DriftY-*VyAccel)/(2*Frq_Update);
	}
	else {
		 if((mmIn[0]>150 && mmIn[0]<4000) ||(mmIn[1]>150 && mmIn[1]<4000)){
			    static uint32_t TickLast;
			    uint32_t Time=(TickNow-TickLast);
				if(mmIn[0]<=mmIn[1])MovingSpeedToF[0]=(float)(mmIn[0]-mmInLast[0])*100/(float)Time;
				else MovingSpeedToF[0]=((float)mmInLast[1]-(float)mmIn[1])*100/(float)Time; //right
				if(MovingSpeedToF[0]>(*VxAccel+50))MovingSpeedToF[0]=*VxAccel+50;
				else if(MovingSpeedToF[0]<(*VxAccel-50))MovingSpeedToF[0]=*VxAccel-50;
				else TofXLocked=1; //LockOK

				*VxAccel+=(MovingSpeedToF[0]-*VxAccel)/(2*Frq_Update);
				 TickLast=TickNow;
			     mmInLast[0]=mmIn[0];
			     mmInLast[1]=mmIn[1];
	     }

		 if((mmIn[2]>150 && mmIn[2]<4000)){
			    static uint32_t TickLast;
			    uint32_t Time=(TickNow-TickLast);

				MovingSpeedToF[1]=((float)mmIn[2]-(float)mmInLast[2])*100/(float)Time; //Convert to cm/s
				if(MovingSpeedToF[1]>(*VyAccel+50))MovingSpeedToF[1]=*VyAccel+50;
				else if(MovingSpeedToF[1]<(*VyAccel-50))MovingSpeedToF[1]=*VyAccel-50;
				else TofYLocked=1;
				*VyAccel+=(MovingSpeedToF[1]-*VyAccel)/(2*Frq_Update);
				 TickLast=TickNow;
					mmInLast[2]=mmIn[2];
		 }
	}

	return ReVal;
}

