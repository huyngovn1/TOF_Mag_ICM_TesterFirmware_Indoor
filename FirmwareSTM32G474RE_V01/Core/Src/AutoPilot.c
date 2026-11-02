#include <stdio.h>
#include <math.h>
#include "Fc.h"
#include "AutoPilotInput.h"
#include "AutoPilot.h"
#include "Kalman.h"
#include "Global.h"

uint8_t HIGH_THRO_HOVER_RTH_EN=0;

uint8_t UBX_UartBuf1[36],MAG_UartBuf1[12],ALTI_UartBuf1[12];
uint8_t UBX_UartBuf[36],MAG_UartBuf[12],ALTI_UartBuf[12];
uint8_t UBX_UartBuf3[36],ALTI_UartBuf3[12],MAG_UartBuf3[36];
extern void  Write_Flash_Config(uint32_t Adr,uint32_t Val32);
extern uint8_t RingLimit(float *x, float *y,float Limit,float *RingOut);
extern char temUsart[];
extern uint8_t RxBuffer1[], RxBuffer3[],RxBuffer2[],Armed,TxHotKey;
extern float  GyroLP[],Angle[],AngleFlybar[],TxMoveRing;
float HoverAngle[2];
float angle_HMC;
extern  float Z_GyroRate,I_AngleFlybar[],accelNonOfset[],TxMoveDelay[],accel_offsetZ;
extern int16_t RXVal[],TxMove2[],TxMove[];
int16_t RXValLast[6],MagOffset[3];
extern int16_t IDLE_THRO;
extern uint32_t TxIdleTime,RxLostTime,TimOutFM24,CountTest;
extern uint16_t ThroNow;
uint16_t GPS_PosCm=0;
extern uint16_t LOST_TX_RTH,RTH_TAIL_MODE;
extern  uint16_t FM24_AltiSet, FM24_AltiMode,DistanceFM24Near,DistanceFM24Ground;  // 0-Off 1 AlltiNear
float VXY_IMU=0,XY_DriftRing;
extern uint8_t GPSHomeSet,MagCalib;
extern uint8_t RxBuffer5[];
extern uint32_t clk_MagCalib;
uint8_t GPSHomeWaiting=0,TakeOffOK;
uint32_t HDOP_Last;
uint8_t RTK_HomeSet=0,GPS_Fixed=0;
 GPS_t GPS={
		.TimeoutGPS=99999
 };
 GPS_t GPS1={
		.TimeoutGPS=99999
 };

 GPS_Base_t GPS_Base= {
		 .HDOP=10000,
		 .VDOP=10000,
		 .TimeoutGPS=10000
 };
float AltiFollowGPS_Base=1000;
float sinangle0,sinangle1,cosangle0=1,cosangle1=1,cosangle01=0;
                              float XaccelNotMove,YaccelNotMove,ZaccelNotMove;
                              float AX_horizontal,AY_horizontal,AZ_horizontal;
                              float X_CON_GPS=0,Y_CON_GPS=0;
                              float X_ERR_HOVER=0,Y_ERR_HOVER=0;
                              float  LON_GPS_ERROR_HOVER,LAT_GPS_ERROR_HOVER,X_DriftHover,Y_DriftHover;

                              float VX_BODY_SET=0,VY_BODY_SET=0;
                              float VX_GPS_SET_LAST_POINT=0,VY_GPS_SET_LAST_POINT=0;
                              float HeadVGPS,HeadVBODY,HeadVControl,VBODYControl;
                              float VX_BODY_SET_SOFT=0,VY_BODY_SET_SOFT=0;
                              __IO uint8_t RTK_POSLLH_Update=0,POSLLH_Update=0,BMPOnBoard_Update,ALTI_Update=0,MAG_Update=0;
                              uint32_t MAG_TimOut1,MAG_TimOut3,MAG_TimOut4,MAG_TimOut;
                              __IO uint8_t POSLLH_Update3=0,ALTI_Update3=0,MAG_Update3=0,POSLLH_Select_Update;
                              uint8_t POSLLH_Update1=0,ALTI_Update1=0,MAG_Update1=0,MAG_Update4,GPS_Base_Updated;
                              float VX_GPS=0,VY_GPS=0,VGPS;
                              int32_t LON_GPS_ERROR,LAT_GPS_ERROR;
              float X_BODY_POS_CM_ERROR=0,Y_BODY_POS_CM_ERROR=0;
int32_t LONGTI_OLD,LATI_OLD;
float AltitudeBaroCm,AltiBaroGround,AltitudeBaroCmLP,BaroCmEx;
float AltitudeGPSCm_Old,AltitudeGPSCm_Old2,AltitudeBaroCm_Old,AltitudeBaroCm_Old2,AltitudeBaroCm_Old3;
 float AltiBMP_Cm;
uint32_t GPS_StableTime,BaroNoise=0,BaroMaxMin,GPS_AltiMaxMin;
float V_AltiSlect,V_AltiBaro,V_AltiCmGPS;
int32_t CourseGPS,SpeedGPS,CourseGPS_old;
float I_ANGLE_X=0,I_ANGLE_Y=0;
uint8_t ZoneOver;
uint8_t AltiGPS_SlectMode,AltiGPS_SlectModeLast;
__IO float AutoPitch;
float AltiSlect,AltiHome,AltiHomeGPS,VZ_Accel,V_Alti_TX_Delay,VZ_IMU_Offset=0,OffsetAltiSlectedBaro;
__IO float VX_Accel,VY_Accel;
float AltiRadarHover=100,DistanceHomeCm;
uint32_t ZlocalMaxMin=0, XlocalMaxMin,YlocalMaxMin;
float Alti_CON,ErrorAltiIMU,PercomSpeed;
float OffsetGPS_BaroRAW;
uint8_t RadarLogic;
int16_t  GPS_GAIN=100,GPS_PGAIN=50,GPS_IGAIN=50,GPS_DGAIN=50 ;
int16_t  ALTI_GAIN=100,ALTI_PGAIN=50,ALTI_IGAIN=50,ALTI_DGAIN=50,GPS_BRAKE=10, WAYPOINT_ACC=10;
uint32_t TimeOutCabMAG,MagCab_Count;
extern float sinHMC,cosHMC;
uint8_t GPSLowSignal;
uint8_t GoingHomeStep=0;
uint8_t FlyMode=3,FlyingStatus,GPSAltiSw=0;
uint32_t CountLoop50Hz=0,CountGPS_ModeTime=0,TimOutBaro,CountAngle_ModeTime;
int16_t TakeoffAngle;
float  LongtiScale=1.07f;
float OffsetSwModeAlti,Offset_Baro_gps,Offset_GPS_Baro;
 uint32_t TimOutFM24NearVail;
extern uint8_t FlyModeBank[];
extern uint32_t  MavLinkLostTime,InitDoneTimeMs;
float ThroHover=RX_CENTRE;
extern uint8_t GPSBoardPlug;
Kalman_f KalManBaro = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};

Kalman_f KalmanDiftX = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};
Kalman_f KalmanDiftY = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};
Kalman_f KalmanDiftZ = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};

Kalman_f KalmanBrakeX = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};
Kalman_f KalmanBrakeY = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};

float BrakeX,BrakeY;

float Kalman_XError,Kalman_YError;
Kalman_f Kalman_VZ_IMU = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};

double KalmanAltiIMU;

float VxGPSBody,VyGPSBody;
Kalman_f VX_GPS_BODY = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};

Kalman_f VY_GPS_BODY = {
        .Q = KALMAN_Q,
        .Q_bias = KALMAN_BIAS,
        .R_measure = KALMAN_R
};
float AxGPS_Body,AyGPS_Body;

int32_t FM24_NP100_RadarFindNear(uint8_t *Bufin) {
   for(uint8_t i=1;i<125;i++)if(Bufin[i]>1){

      float DistanNear=((float)i-(float)Bufin[i]/44.0f )*8.25f;

      return  (int32_t)DistanNear +6;
   }
   // MaxRSSI=0;
   // for(char i=0;i<125;i++)if((Bufin[i]+Bufin[i+1])> MaxRSSI){ MaxRSSI=Bufin[i]+Bufin[i+1]; MaxRSSIPos=i;}
   // For example: if the amplitude reaches the maximum peak, caculate the central point of the maximum peak. The central point will be the n-th data, then multiply by 0.126.
   //eg. If the spectral line amplitude forms peaks between 14th to 25th: The distance of the point should be: (14+(25-14)/2)*0.126=2.457 (Unit:m)

   return  0;
}

//------------------For Indoor Test Only--------------------
void VisualGPSTest(void){
	static uint32_t countVis=0;
	GPS.HDOP=HDOP_Last=1000;
	GPS.VDOP=1000;
	GPS.TimeoutGPS=100;
	GPS.FrameRateMs=100;
	GPS.AltitudeCm=2000;
	GPS.Lat=10000000;
	GPS.Lon=100000000;
	if(++countVis>=50){countVis=0;POSLLH_Select_Update=1;}

   // VX_GPS_BODY.Out=0;
   // VY_GPS_BODY.Out=0;
   // AxGPS_Body=AyGPS_Body=0;
   // GPSHomeSet=1;
/*    if(PIT_CON>7200)Alti_CON-=0.01f;
    else if(PIT_CON<4800)Alti_CON+=0.01f;*/
   // RxLostTime=0;

}

uint8_t GPS_NAV_POSLLH_Decoder(GPS_t* GPSin, uint8_t *Buf){   //UBX-NAV-PVT (0x01 0x07) FUll
	   if(Buf[2]!=0x01 ||Buf[3]!=0x02){return 0;}
       uint8_t CKA=0,CKB=0;
        for(uint8_t i=2;i<34;i++){
          CKA+=Buf[i];
          CKB+=CKA;
         }
        uint16_t UBXOffset=Buf[4]|(Buf[5]<<8);
        if((CKA!=Buf[34])||(CKB!=Buf[35]) ||UBXOffset!=28) {return 0;}
        GPSin->TimeWeek=Buf[6]|(Buf[7]<<8)|(Buf[8]<<16)|(Buf[9]<<24);
        GPSin->Lon=Buf[10]|(Buf[11]<<8)|(Buf[12]<<16)|(Buf[13]<<24);   // E-7/Dig <=>1.1cm in VN
        GPSin->Lat=Buf[14]|(Buf[15]<<8)|(Buf[16]<<16)|(Buf[17]<<24);   // E-7/Dig <=>1.1cm in VN
        int32_t AltitudeMm= Buf[18]|(Buf[19]<<8)|(Buf[20]<<16)|(Buf[21]<<24);
        GPSin->HDOP=	Buf[26]|(Buf[27]<<8)|(Buf[28]<<16)|(Buf[29]<<24);  //Horizontal Accuracy Estimate [mm]
        GPSin->VDOP=	Buf[30]|(Buf[31]<<8)|(Buf[32]<<16)|(Buf[33]<<24);  //Vertical Accuracy Estimate [mm]
        GPSin->AltitudeCm=(float)AltitudeMm/10.0f;
        if(GPSin->Lon&&GPSin->Lat&&(GPSin->HDOP<=20000)&&(GPSin->VDOP<=20000) && GPSin->AltitudeCm>-20000) { GPSin->TimeoutGPS=0; return 1; }
        else {
        	return 0;
        }
}


uint8_t GPS_Base_Decoder(GPS_Base_t *GPS_Base, uint8_t *Buf){   //UBX-NAV-PVT (0x01 0x07) FUll
	if(Buf[2]!=0x01 ||Buf[3]!=0x02 ||Buf[1]!=0x63){return 0;}
    uint8_t CKA=0,CKB=0;
     for(uint8_t i=2;i<44;i++){
       CKA+=Buf[i];
       CKB+=CKA;
      }
     uint16_t UBXOffset=Buf[4]|(Buf[5]<<8);
     if((CKA!=Buf[44])||(CKB!=Buf[45]) ||UBXOffset!=28) {return 0;}

     GPS_Base->TimeWeek=Buf[6]|(Buf[7]<<8)|(Buf[8]<<16)|(Buf[9]<<24);
     GPS_Base->Lon=Buf[10]|(Buf[11]<<8)|(Buf[12]<<16)|(Buf[13]<<24);   // E-7/Dig <=>1.1cm in VN
     GPS_Base->Lat=Buf[14]|(Buf[15]<<8)|(Buf[16]<<16)|(Buf[17]<<24);   // E-7/Dig <=>1.1cm in VN
     int32_t AltitudeMm= Buf[18]|(Buf[19]<<8)|(Buf[20]<<16)|(Buf[21]<<24);
     GPS_Base->HDOP=	Buf[26]|(Buf[27]<<8)|(Buf[28]<<16)|(Buf[29]<<24);  //Horizontal Accuracy Estimate [mm]
     GPS_Base->VDOP=	Buf[30]|(Buf[31]<<8)|(Buf[32]<<16)|(Buf[33]<<24);  //Vertical Accuracy Estimate [mm]
     GPS_Base->AltitudeCm=(float)AltitudeMm/10.0f;

     GPS_Base->xMag=((int16_t)Buf[34]<<8)|Buf[35];
     GPS_Base->yMag=((int16_t)Buf[36]<<8)|Buf[37];
     GPS_Base->zMag=((int16_t)Buf[38]<<8)|Buf[39];

     GPS_Base->AltitudemmBaro=((int32_t)Buf[40]<<24)|((int32_t)Buf[41]<<16)|((int32_t)Buf[42]<<8)|Buf[43];

     if(GPS_Base->Lon&&GPS_Base->Lat&&(GPS_Base->HDOP<=20000)&&(GPS_Base->VDOP<=20000) && GPS_Base->AltitudeCm>-20000) {
    	 GPS_Base->VLonGPSBase=(float)(GPS_Base->Lon-GPS_Base->LonLast)*50.0f/(float)(GPS_Base->TimeoutGPS);
    	 GPS_Base->VLatGPSBase=(float)(GPS_Base->Lat-GPS_Base->LatLast)*50.0f/(float)(GPS_Base->TimeoutGPS);
    	 GPS_Base->VZ_GPS_Base=(GPS_Base->AltitudeCm-GPS_Base->AltitudeCmLast)*50.0f/(float)(GPS_Base->TimeoutGPS);

    	 GPS_Base->LonLast=GPS_Base->Lon;
    	 GPS_Base->LatLast=GPS_Base->Lat;
    	 GPS_Base->AltitudeCmLast=GPS_Base->AltitudeCm;
    	 GPS_Base->TimeoutGPS=0;
         return 1;
     }
     else {
     	return 0;
     }
}

uint8_t CompassUartDecoder(uint8_t *Buf,int16_t *MagRaw) {
        uint8_t CKA=0,CKB=0;
        for(uint8_t i=4;i<10;i++){
          CKA=CKA+Buf[i];
          CKB=CKB+CKA;
         }
         if(CKA==Buf[10]&&CKB==Buf[11]){
             MagRaw[0]=(Buf[4]<<8)|Buf[5];
             MagRaw[1]=(Buf[6]<<8)|Buf[7];
             MagRaw[2]=(Buf[8]<<8)|Buf[9];
            return 1;
         }
        else return 0;
}


uint8_t BaroUartDecoder(uint8_t *Buf, float *AltiCm) {
        uint8_t CKA=0,CKB=0;
        for(uint8_t i=4;i<10;i++){
          CKA=CKA+Buf[i];
          CKB=CKB+CKA;
         }
         if(CKA==Buf[10]&&CKB==Buf[11]){
            int32_t AltitudeBaroMm=(Buf[4]<<24)|(Buf[5]<<16)|(Buf[6]<<8)|Buf[7];
            *AltiCm= (float)AltitudeBaroMm/10.0f;
            return 1;
         }
        else return 0;
}


uint32_t BaroNoiseMetter10Hz(int32_t AltiCm) { //10Hz
  uint32_t Noise;
  static int32_t AltiCmLast[30];
  for(uint8_t i=29;i>0;i--)AltiCmLast[i]=AltiCmLast[i-1];
  AltiCmLast[0]=AltiCm;
  int32_t Sum=0;
  for(uint8_t i=0;i<=28;i++){
    int32_t Delta=AltiCmLast[i+1]-AltiCmLast[i]; if(Delta<0)Delta=0-Delta;
    Sum+=Delta;
  }
  int32_t AltiChange=AltiCmLast[0]-AltiCmLast[29]; if(AltiChange<0)AltiChange=0-AltiChange;
  Noise=Sum-AltiChange;
  //sprintf(temUsart, "ANoise:%d_%d_%d\n",AltiChange,Sum,Noise); sendUsart1(temUsart);
  return Noise;
}

uint32_t X_IMUMinMetter10Hz(int32_t AltiCm) { // 5s last
  static int32_t IMUCmLast[50];
  int32_t Max,Min;
  for(uint8_t i=49;i>0;i--)IMUCmLast[i]=IMUCmLast[i-1];
  IMUCmLast[0]=AltiCm;
  Min=Max=IMUCmLast[0];
  for(uint8_t i=1;i<=49;i++){
    if(IMUCmLast[i]<Min)Min=IMUCmLast[i];
    if(IMUCmLast[i]>Max)Max=IMUCmLast[i];
  }
  return Max-Min;
}

uint32_t Y_IMUMinMetter10Hz(int32_t AltiCm) { // 5s last
  static int32_t IMUCmLast[50];
  int32_t Max,Min;
  for(uint8_t i=49;i>0;i--)IMUCmLast[i]=IMUCmLast[i-1];
  IMUCmLast[0]=AltiCm;
  Min=Max=IMUCmLast[0];
  for(uint8_t i=1;i<=49;i++){
    if(IMUCmLast[i]<Min)Min=IMUCmLast[i];
    if(IMUCmLast[i]>Max)Max=IMUCmLast[i];
  }
  return Max-Min;
}

uint32_t Z_IMUMinMetter10Hz(int32_t AltiCm) { // 5s last
  static int32_t IMUCmLast[50];
  int32_t Max,Min;
  for(uint8_t i=49;i>0;i--)IMUCmLast[i]=IMUCmLast[i-1];
  IMUCmLast[0]=AltiCm;
  Min=Max=IMUCmLast[0];
  for(uint8_t i=1;i<=49;i++){
    if(IMUCmLast[i]<Min)Min=IMUCmLast[i];
    if(IMUCmLast[i]>Max)Max=IMUCmLast[i];
  }
  return Max-Min;
}

uint32_t BaroMaxMinMetter10Hz(int32_t AltiCm) { // 3s
  static int32_t AltiCmLast[30];
  int32_t Max,Min;
  for(uint8_t i=29;i>0;i--)AltiCmLast[i]=AltiCmLast[i-1];
  AltiCmLast[0]=AltiCm;
  Min=Max=AltiCmLast[0];
  for(uint8_t i=1;i<=29;i++){
    if(AltiCmLast[i]<Min)Min=AltiCmLast[i];
    if(AltiCmLast[i]>Max)Max=AltiCmLast[i];
  }
  return Max-Min;
}


uint32_t GPS_AltiMaxMinMetter10Hz(int32_t AltiCm) { // 3s
  static int32_t DiftCmLast[30];
  int32_t Max,Min;
  for(uint8_t i=29;i>0;i--)DiftCmLast[i]=DiftCmLast[i-1];
  DiftCmLast[0]=AltiCm;
  Min=Max=DiftCmLast[0];
  for(uint8_t i=0;i<=29;i++){
    if(DiftCmLast[i]<Min)Min=DiftCmLast[i];
    if(DiftCmLast[i]>Max)Max=DiftCmLast[i];
  }
  return Max-Min;
}



uint32_t FirstTimeLoadOK=0,CountTimeGPSFixedOK=0,CountTimeBaroUpdateOK=0,CountTimeMagUpdated=0;
void FirstTimeLoad(void){
	GPS.VDOP=100000;
	GPS.HDOP=1000000;
  ALTI_Update=0;
  POSLLH_Update=0;
  MAG_Update=0,
  FirstTimeLoadOK=1;
}

void FirstTimeGPSFixed(void){
  AltitudeGPSCm_Old = AltitudeGPSCm_Old2=GPS.AltitudeCm;
  LONGTI_OLD=GPS.Lon;
  LATI_OLD=GPS.Lat;
}
void FirstTimeBaroUpdate(float AltitudeBaroCm){
  AltitudeBaroCm_Old=AltitudeBaroCm_Old2=AltitudeBaroCm;
  AltitudeBaroCm_Old3=AltitudeBaroCm_Old2;
  KalManBaro.Out=AltitudeBaroCm;

  KalmanAltiIMU=AltitudeBaroCm;
  KalManBaro.Out=AltitudeBaroCm;
  AltiSlect=AltitudeBaroCm;  OffsetSwModeAlti=0;
  Alti_CON=AltiSlect;
  VZ_Accel=0;
  AltiGPS_SlectMode=0;
  AltiHome=AltitudeBaroCm;
}

uint32_t countTimeBaroOnning=0,TimeAltiNotMoveTx=0,CountHoverVailRadar=0;
int16_t ThroManual=RX_CENTRE, OffsetSoftPIT=0;
void BaroModeStoping(void){
         V_AltiSlect=V_AltiBaro;
         AltitudeBaroCm_Old=AltitudeBaroCm_Old2=AltitudeBaroCm;
	 AltiSlect=AltitudeBaroCm;  AltitudeBaroCm_Old3=AltitudeBaroCm_Old2;
	 KalManBaro.Out=AltitudeBaroCm;
         AltiGPS_SlectMode=0; OffsetSwModeAlti=0;
         KalmanAltiIMU=AltiSlect;
         ErrorAltiIMU=0;
	 Alti_CON=AltiSlect;
//         if(AutoPitch>=0.5f)AutoPitch-=0.5f;   // Soft return
//         else if (AutoPitch<=(-0.5f))AutoPitch+=0.5f;
         countTimeBaroOnning=0;
         V_Alti_TX_Delay=0;
         VZ_Accel=0;
//         PIT_CON=RC_ALTITUDE_CHANNEL+OffsetSoftPIT;
         if(OffsetSoftPIT<-1)OffsetSoftPIT+=2; else if(OffsetSoftPIT>1)OffsetSoftPIT-=2;

         ThroManual=RC_ALTITUDE_CHANNEL;
         Offset_Baro_gps=Offset_GPS_Baro=0;
}
extern int32_t UltraSonicMm,FramaRateSonic,TimOutSonicVail,SonicCountVail10Hz;
 uint8_t UltraSonicUpdated, RadarZ_Update;
extern  int32_t SonicMm10Hz;
void BaroModeOnning(void){  // 500Hz Loop
                             float TxAltiUpDown=((float)(RC_ALTITUDE_CHANNEL-(RX_CENTRE)))/4.0f;

                               if(TxAltiUpDown>125.0f)TxAltiUpDown-=120.0f;
                               else  if(TxAltiUpDown<-125.0f)TxAltiUpDown+=120.0f;
                               else  TxAltiUpDown=0;
                               if(AltitudeBaroCmLP>(AltiHome+LIMIT_ALTITUDE)){ // AltiLImit
                                 if(TxAltiUpDown>-50)TxAltiUpDown=-50;
                               }

                               if(TxAltiUpDown<0)TxAltiUpDown*=0.67f; // Slow Down 2.0m/s
                               V_Alti_TX_Delay+=(TxAltiUpDown-V_Alti_TX_Delay)/((float)Frq_Update*1.0f);

                               if(TxAltiUpDown>(V_Alti_TX_Delay+0.1f))V_Alti_TX_Delay+=0.1f;
                               else if(TxAltiUpDown<(V_Alti_TX_Delay-0.1f))V_Alti_TX_Delay-=0.1f;
                               else V_Alti_TX_Delay=TxAltiUpDown;

                               if(V_Alti_TX_Delay==0)TimeAltiNotMoveTx++; else TimeAltiNotMoveTx=0;


                               if(AltiGPS_SlectMode && GPS.AltitudeCm>-20000 && (GPS.VDOP<VHDOP_HOME)&&(GPS.HDOP<VHDOP_HOME) && GPS.TimeoutGPS<250){ //GPS alti running; && GPS.AltitudeCm<300000
                                    V_AltiSlect=V_AltiCmGPS;
				                    AltiSlect=GPS.AltitudeCm+Offset_Baro_gps;
                                    if((VGPS<(GPS.VDOP/24+50) && (BaroMaxMin<(GPS_AltiMaxMin+30)) && cosangle01>0.99026f) || (GPS_AltiMaxMin>(BaroMaxMin+200)))AltiGPS_SlectMode=0;

                                    VZ_Accel+=(V_AltiSlect-VZ_Accel)/((float)Frq_Update*1.0f);
                                    KalmanAltiIMU+=(AltiSlect-KalmanAltiIMU)/(Frq_Update*1.0);

                                   // OffsetAltiSlectedGPS+=((AltiSlect-KalmanAltiIMU)-OffsetAltiSlectedGPS)*0.002f;  // Khu sai offset Baro Va GPS
                                    Offset_GPS_Baro=AltiSlect-AltitudeBaroCmLP +(OffsetAltiSlectedBaro*0.94f); // Soft switch
                               }
                               else if(AltitudeBaroCmLP>-20000 ){    //Baro && AltitudeBaroCmLP<300000
                                    V_AltiSlect=V_AltiBaro;
				                    AltiSlect=AltitudeBaroCmLP+Offset_GPS_Baro;
                                    if(GPSHomeSet && GPS.VDOP<(VHDOP_HOME-500) &&GPS.HDOP<(VHDOP_HOME-500) && GPS.TimeoutGPS<150 && TakeOffOK){
                                       if((VGPS>(GPS.VDOP/24 +100))||(BaroMaxMin>(GPS_AltiMaxMin+50)) || cosangle01<0.98480f) AltiGPS_SlectMode=1;	 // Alt theo GPS >1m/s
                                    }
                                    VZ_Accel+=(V_AltiSlect-VZ_Accel)/((float)Frq_Update*4.0f);
                                    KalmanAltiIMU+=(AltiSlect-KalmanAltiIMU)/(Frq_Update*4.0);

                                    if(TimeAltiNotMoveTx>1000 && VBODYControl==0 && Armed>=2)OffsetAltiSlectedBaro+=((AltiSlect-KalmanAltiIMU)-OffsetAltiSlectedBaro)*0.001f;
                                    Offset_Baro_gps=AltiSlect-GPS.AltitudeCm-(OffsetAltiSlectedBaro*0.94f);
                               } else { //Fail Altitude sensor
                            	   //Armed=0;
//                            	   PIT_CON=RC_ALTITUDE_CHANNEL-480;
                            	   return;
                               }

                                 VZ_Accel+=(AZ_horizontal)*(SCALE_ACCEL/(float)Frq_Update);

                                 if(VZ_Accel>(V_Alti_TX_Delay+400))VZ_Accel=(V_Alti_TX_Delay+400); //Error IMU
                                 else if(VZ_Accel<(V_Alti_TX_Delay-400))VZ_Accel=(V_Alti_TX_Delay-400);

                                 KalmanAltiIMU+=(double)(VZ_Accel)/(double)Frq_Update;

                                 if(KalmanAltiIMU<(AltiSlect-400)){ //Error IMU
                                	 KalmanAltiIMU=(AltiSlect-400);
                                 }
                                 else  if(KalmanAltiIMU>(AltiSlect+400))KalmanAltiIMU=(AltiSlect+400);

                                 //-------------------Len xuong deu hon---------------------------
                                 //else if(KalmanAltiIMU<(AltiSlect-300)){ Offset_Baro_gps-=0.1f; Offset_GPS_Baro-=0.1f;}
                                 //else if(KalmanAltiIMU>(AltiSlect+300)){ Offset_Baro_gps+=0.1f; Offset_GPS_Baro+=0.1f;}

                                 //-------------------High Vibration, Error IMU - Itseft go Up-------------------------
#ifdef DETECTOR_IMU_ERR
                                 static uint32_t CountTimeErr=0;
                            	 if(AZ_horizontal<-400 && VZ_Accel<-200 &&  V_AltiCmGPS>200 && V_AltiBaro >200 && V_Alti_TX_Delay<=0 && ThroNow>=(MaxThro-800)){
                            		 PIT_CON=RC_ALTITUDE_CHANNEL-480;
                            		 if(++CountTimeErr>=(2*Frq_Update))Armed=0; //2s goUp Auto Cutoff
                            	 } else CountTimeErr=0;

                            	  if((VZ_Accel>(V_Alti_TX_Delay+500))||(VZ_Accel<(V_Alti_TX_Delay-500)))Armed=0;
#endif

                              Alti_CON+=V_Alti_TX_Delay/(float)(Frq_Update);
                              ErrorAltiIMU=Alti_CON-KalmanAltiIMU;// cm

                              if(ErrorAltiIMU>250){ ErrorAltiIMU=250; Alti_CON=(KalmanAltiIMU+250);  }
                              else if(ErrorAltiIMU<-250){ErrorAltiIMU=-250; Alti_CON=(KalmanAltiIMU-250);}

                              PercomSpeed= (VX_Accel*sinangle0) + (VY_Accel*sinangle1);// Bu theo toc do khi canh nghieng
                              if(PercomSpeed>1000)PercomSpeed=1000; else if(PercomSpeed<-1000)PercomSpeed=-1000;

                              float Verror=(V_Alti_TX_Delay-VZ_Accel)*11.0f;

			                  AutoPitch=(ErrorAltiIMU*(float)ALTI_PGAIN*(30.0f/50.0f)) + Verror + (PercomSpeed*2.0f);

                              float tmpAZ=AZ_horizontal*(0.25f*SCALE_ACCEL);
                              AutoPitch-=tmpAZ;

#ifdef HELICOPTER_KIT
                              if(AutoPitch>0)AutoPitch=AutoPitch/cosangle01; // Bu goc nghieng
#else
                              AutoPitch=(AutoPitch+1600.0f)/cosangle01 -1600.0f; // Bu goc nghieng
#endif
                              if(AutoPitch<-3200){AutoPitch=-3200; }
                              else if(AutoPitch>3200){AutoPitch=3200; }  //85%

                              if(++countTimeBaroOnning<(Frq_Update)) { // Sau khi lai Rescue 2s
                                     KalmanAltiIMU=AltiSlect;
				                     Alti_CON=AltiSlect;
                                     AutoPitch=AutoPitch*(float)countTimeBaroOnning/(float)(Frq_Update);
                              }
                              AutoPitch=AutoPitch*(float)ALTI_GAIN/100.0f ;

//                              PIT_CON=(int16_t)AutoPitch + ThroManual;
                              if(ThroManual>(RX_CENTRE+1))ThroManual-=2; else if(ThroManual<(RX_CENTRE-1))ThroManual+=2; // Soft Switch Mode
//                              OffsetSoftPIT=PIT_CON-RC_ALTITUDE_CHANNEL;
}

//--------------------------Going Home-------------------
uint16_t GoingHomeMode=0,PERCENT_BAT_RTH=20,MAX_DISTANCE_FLY=100;
int16_t PercentBat=100;
uint8_t LowBatRTHEn=0;
float mAhComsum;
uint16_t  PercientBat,VCellAlarm1=25,VCellAlarm2=20,VCellAlarm3=15,VCellBatery=4200,Cell=0;
uint16_t RTH_SPEED_CM=400,MinAltiRTH=1000,SPEED_UP_DOWN_GOHOME=150;
uint8_t WaypointRepeat=0;
int32_t  Vbat;
uint8_t SetHeading(int16_t HeadingSet, float *TailControl){
  uint8_t ReVal=0;
  int16_t HeadDeg=(int16_t)(angle_HMC*ONE_RAD);
  int16_t ErrorHead=HeadDeg-HeadingSet;
   if(ErrorHead<=-180)ErrorHead+=360; else if(ErrorHead>=180)ErrorHead-=360;

   if(ErrorHead>30)*TailControl+=1.2f; else if(ErrorHead<-30)*TailControl-=1.2f;
   else if((ErrorHead>6)||(ErrorHead<-6)) { *TailControl+=(float)ErrorHead/50.00f;  }
   else  ReVal=1;
   return ReVal;
}

uint8_t SetHeadtoPoint(int32_t Lon, int32_t Lat,uint16_t Heading, float *TailControl){
    if(Heading<360){
        float VXGPS_SET,VYGPS_SET;
        VXGPS_SET=(float)(Lon-(int32_t)GPS.LonSet);
        VYGPS_SET=(float)(Lat-(int32_t)GPS.LatSet);

       float AngletoHomePoint = atan2f(VXGPS_SET,VYGPS_SET);
       float ErrHeadToHome=angle_HMC-AngletoHomePoint + (float)Heading*ONE_DEG;
       if(ErrHeadToHome>Pi)ErrHeadToHome-=2*Pi; else if(ErrHeadToHome<-Pi)ErrHeadToHome+=2*Pi;
       if(ErrHeadToHome>Pi)ErrHeadToHome-=2*Pi; else if(ErrHeadToHome<-Pi)ErrHeadToHome+=2*Pi;

       if(ErrHeadToHome> (ONE_DEG*45.0f))*TailControl+=1.2f;
       else if(ErrHeadToHome< (ONE_DEG*-45.0f))*TailControl-=1.2f; // 60d/s
       else  if((ErrHeadToHome> (ONE_DEG*8.0f))|| (ErrHeadToHome< (ONE_DEG*-8.0f))) { *TailControl+=ErrHeadToHome;}
       else return 1;
    }
    return 0;
}
/*
-SpeedCms La van toc di qua diem do
*/
float  GotoOnePoint(int32_t Lon, int32_t Lat,float SpeedCms,float Acc,uint32_t HoldMs){ // 50Hz loop
    float DLAT,DLON;
    float BrakeXtmp,BrakeYtmp;
    float tmpRingLimit;

    DLON=(float)(Lon-(int32_t)GPS.LonSet);
    DLAT=(float)(Lat-(int32_t)GPS.LatSet);

    float DxCm=DLON*LongtiScale;
    float DyCm=DLAT*1.11f;
    float DistanceCm=sqrtf((float)((DxCm*DxCm)+(DyCm*DyCm)));
    if(DistanceCm==0){
      VX_BODY_SET=0; VY_BODY_SET=0;
      return 0;
    }

    float DistanceRingCm=(SpeedCms*SpeedCms)/(Acc*20.0f);

    float VxGPScm,VyGPScm,Speed;
    if(DistanceCm>DistanceRingCm){ // Xa
            VxGPScm=(DxCm/DistanceCm)*SpeedCms;
            VyGPScm=(DyCm/DistanceCm)*SpeedCms;

            float Ax=VxGPScm-VX_GPS_SET_LAST_POINT;
            float Ay=VyGPScm-VY_GPS_SET_LAST_POINT;

            if(RingLimit(&Ax,&Ay,Acc/5.0f,&tmpRingLimit)){  // Gioi han tang toc
              VxGPScm=VX_GPS_SET_LAST_POINT+Ax;
              VyGPScm=VY_GPS_SET_LAST_POINT+Ay;
            }
            BrakeXtmp=0; // Gia toc 0
            BrakeYtmp=0;
    }
    else {// Bat dau phanh
            Speed=sqrtf((float)(Acc*(10*2))*DistanceCm);

            VxGPScm=(DxCm/DistanceCm)*Speed;
            VyGPScm=(DyCm/DistanceCm)*Speed;

            BrakeXtmp=(DxCm/DistanceCm)*Acc*-13.1f;
            BrakeYtmp=(DyCm/DistanceCm)*Acc*-13.1f;

            BrakeXtmp=(((float)BrakeXtmp*cosHMC)- ((float)BrakeYtmp*sinHMC));	// Converto heading to Body
            BrakeYtmp=(((float)BrakeYtmp*cosHMC)+ ((float)BrakeXtmp*sinHMC));
    }

           GPS.LonSet+=(double)(VxGPScm/LongtiScale/50.0f);
           GPS.LatSet+=(double)(VyGPScm/(50.0f*1.11f));

           VX_BODY_SET=(((float)VxGPScm*cosHMC)- ((float)VyGPScm*sinHMC));	// Converto Cm/s
           VY_BODY_SET=(((float)VyGPScm*cosHMC)+ ((float)VxGPScm*sinHMC));

//           if(DistanceCm<10){
//             VX_BODY_SET=0; VY_BODY_SET=0;
//             BrakeXtmp=0; BrakeYtmp=0;
//           }
                               float Delta=(VX_BODY_SET-VX_BODY_SET_SOFT)*0.1f;
                               if(Delta>5)Delta=5; else if(Delta<-5)Delta=-5; // Maxspeed change 250cm/ss
                               VX_BODY_SET_SOFT+=Delta;

                               Delta=(VY_BODY_SET-VY_BODY_SET_SOFT)*0.1f;
                               if(Delta>5)Delta=5; else if(Delta<-5)Delta=-5;
                               VY_BODY_SET_SOFT+=Delta;

                               Delta=(BrakeXtmp-BrakeX)*0.1f; if(Delta>25)Delta=25; else if(Delta<-25)Delta=-25;
                               BrakeX+=Delta;

                               Delta=(BrakeYtmp-BrakeY)*0.1f; if(Delta>25)Delta=25; else if(Delta<-25)Delta=-25;
                               BrakeY+=Delta;


                               VX_GPS_SET_LAST_POINT=VxGPScm;
                               VY_GPS_SET_LAST_POINT=VyGPScm;

    return DistanceCm;
}



WayPoint WayPoints={.Speed=5};

uint8_t SPRAY_DRONE=1;

void ResetWaypoint(void){
  WayPoints.Repeat=WaypointRepeat;
  WayPoints.NumberPoints=0;
  WayPoints.PointFlying=0;
}

uint16_t WayPointsFlying(WayPoint *BankFly){ // 50Hz Loop
  if(BankFly->PointFlying  < BankFly->NumberPoints)
  {
    float DistanceToPointCm=GotoOnePoint(BankFly->Lon[BankFly->PointFlying], BankFly->Lat[BankFly->PointFlying],(float)RTH_SPEED_CM,(float)WAYPOINT_ACC,0);
    if(DistanceToPointCm<3.0f) (BankFly->PointFlying)+=1;
    else  {
    	if(BankFly->PointFlying>0){
    	  int32_t DLon=BankFly->Lon[BankFly->PointFlying]-BankFly->Lon[BankFly->PointFlying-1];
    	  int32_t DLat=BankFly->Lat[BankFly->PointFlying]-BankFly->Lat[BankFly->PointFlying-1];

    	  float DistanceLastToNextPointCm=sqrtf((DLat*DLat*(1.11f*1.11f))+(DLon*DLon *LongtiScale*LongtiScale));
    	  if(DistanceToPointCm>300 &&DistanceLastToNextPointCm>1000) {
    		   uint16_t HeadingSet=0;
    	       float AngleToPoint = atan2f((float)DLon,(float)DLat);
    	       float ErrHeadToHome=angle_HMC-AngleToPoint + ((float)HeadingSet*ONE_DEG);
    	       if(ErrHeadToHome>Pi)ErrHeadToHome-=2*Pi; else if(ErrHeadToHome<-Pi)ErrHeadToHome+=2*Pi;
    	       if(ErrHeadToHome>Pi)ErrHeadToHome-=2*Pi; else if(ErrHeadToHome<-Pi)ErrHeadToHome+=2*Pi;

    	       if(SPRAY_DRONE  && (ErrHeadToHome>Pi/2.0f || ErrHeadToHome<Pi/-2.0f))HeadingSet=180;  // Bay lui
    		   SetHeadtoPoint(BankFly->Lon[BankFly->PointFlying],BankFly->Lat[BankFly->PointFlying],HeadingSet,&AngleFlybar[2]);
    	  }
    	}
    	else  SetHeadtoPoint(BankFly->Lon[0],BankFly->Lat[0],0,&AngleFlybar[2]); // Point dau tien
    }
    return 1;
  }
  else  {
    if(WaypointRepeat){BankFly->PointFlying=0; return 1; }
       else return 0;
    }
}


void HoverGPS(void){
                                            VX_BODY_SET=0; VY_BODY_SET=0;
                                            VX_GPS_SET_LAST_POINT=0; VY_GPS_SET_LAST_POINT=0;

                                            BrakeX-=BrakeX/((float)Frq_Update*0.04f); // After Brake
                                            BrakeY-=BrakeY/((float)Frq_Update*0.04f);

                                            VX_BODY_SET_SOFT-=VX_BODY_SET_SOFT/((float)Frq_Update*0.02f); // After Brake
                                            VY_BODY_SET_SOFT-=VY_BODY_SET_SOFT/((float)Frq_Update*0.02f);

                                            LON_GPS_ERROR_HOVER=(float)LON_GPS_ERROR;
				                            LAT_GPS_ERROR_HOVER=(float)LAT_GPS_ERROR;
}


void GPS_CyclicModeRuninng(void){  //500Hz Loop
                                if(++CountGPS_ModeTime==1)TxIdleTime=0; // Just Swtiched Mode

                                //----------------Add Point when flying-----------------------------------------
                                static uint32_t CH8HighTime=0;
                                if(RXVal[8]>(RX_CENTRE+1200))CH8HighTime++; else CH8HighTime=0;
                                if(CH8HighTime>(Frq_Update*3) && WayPoints.NumberPoints>=2) FlyMode=WAYPOINT_MODE;

                                if(((PercentBat<PERCENT_BAT_RTH && LowBatRTHEn) ||((ThroHover>(THRO_MAX-480)) && HIGH_THRO_HOVER_RTH_EN))&& (TxMoveDelay[2]==0))FlyMode=GOHOME_MODE;
                                if((TakeOffOK==0 ||GPSHomeWaiting==0)&&(FlyMode==GOHOME_MODE || FlyMode==WAYPOINT_MODE))FlyMode=GPS_MODE; // RTH on Ground

                                if(CountLoop50Hz==1){ // 50hZ Loop  ----Common GPS Mode (move and not Move)----------------
                                   if(TxMove2[2]||TakeOffOK==0){
                                         Kalman_XError-=Kalman_XError/((float)Frq_Update*1.0f);
                                         Kalman_YError-=Kalman_YError/((float)Frq_Update*1.0f);
                                         LON_GPS_ERROR_HOVER-=LON_GPS_ERROR_HOVER/((float)Frq_Update*1.0f);
                                         LAT_GPS_ERROR_HOVER-=LAT_GPS_ERROR_HOVER/((float)Frq_Update*1.0f) ;
                                         X_DriftHover-=X_DriftHover/((float)Frq_Update*1.0f);
                                         Y_DriftHover-=Y_DriftHover/((float)Frq_Update*1.0f);
                                         I_ANGLE_X-=I_ANGLE_X/((float)Frq_Update*1.0f);
                                         I_ANGLE_Y-=I_ANGLE_Y/((float)Frq_Update*1.0f);
                                   }

                                   VXY_IMU=sqrtf((VX_Accel)*(VX_Accel) + ((VY_Accel)*(VY_Accel)));

                                   if((GPS.TimeoutGPS>=1000) ||  (GPS.VDOP>(VHDOP_HOME*2))|| (GPS.HDOP>(VHDOP_HOME*2))) {//For Save Mode Low GPS signal
                                       I_ANGLE_X-=I_ANGLE_X*0.05f;
                                       I_ANGLE_Y-=I_ANGLE_Y*0.05f;
                                       FlyingStatus=NOT_GPS;
                                       VX_BODY_SET=0;
                                       VY_BODY_SET=0;
                                       VX_BODY_SET_SOFT=0;
                                       VY_BODY_SET_SOFT=0;
                                       VBODYControl=0;

                                       VX_Accel-=VX_Accel*0.05f;
                                       VY_Accel-=VY_Accel*0.05f;

                                       Kalman_XError-=Kalman_XError*0.05f;
                                       Kalman_YError-=Kalman_YError*0.05f;

                                       BrakeX-=BrakeX*0.05f;
                                       BrakeY-=BrakeY*0.05f;

                                       LON_GPS_ERROR_HOVER=0; LAT_GPS_ERROR_HOVER=0;
                                       X_DriftHover=0; Y_DriftHover=0;
                                       X_ERR_HOVER=0; Y_ERR_HOVER=0;
                                       AltiGPS_SlectMode=0;
                                       X_BODY_POS_CM_ERROR=0;Y_BODY_POS_CM_ERROR=0;
                                       TxIdleTime=0;

                                       VX_GPS_BODY.Out=0; VY_GPS_BODY.Out=0;
                                       VGPS=0;
                                       AxGPS_Body=0; AyGPS_Body=0;
                                  }
                                  else
                                  {

                                      HeadVControl=atan2f(VX_BODY_SET_SOFT,VY_BODY_SET_SOFT);  //for test &monitor only
                                      RingLimit(&VX_BODY_SET_SOFT, &VY_BODY_SET_SOFT,2000.0f,&VBODYControl);

                                      if(TxIdleTime<10&&(FlyMode==GOHOME_MODE||FlyMode==WAYPOINT_MODE)){ // Lai trong che do GoHome, tu keo sai do gohome ve 0
                                            LON_GPS_ERROR_HOVER-=LON_GPS_ERROR_HOVER/100.0f;
                                            LAT_GPS_ERROR_HOVER-=LAT_GPS_ERROR_HOVER/100.0f;

                                            Kalman_XError-=Kalman_XError/100.0f;
                                            Kalman_YError-=Kalman_YError/100.0f;

                                            I_ANGLE_X-=I_ANGLE_X/100.0f;
                                            I_ANGLE_Y-=I_ANGLE_Y/100.0f;

                                            BrakeX-=BrakeX/((float)Frq_Update*0.02f);
                                            BrakeY-=BrakeY/((float)Frq_Update*0.02f);
                                     }
                                  }     // End Good GPS

                                      //--------------------VGPS Body Fillter
//                                      float DxGPS,DyGPS,Dxy;
//                                      DxGPS=VX_GPS_BODY.Out-VX_GPS_BODYf;
//                                      DyGPS=VY_GPS_BODY.Out-VY_GPS_BODYf;
//
//                                      if(RingLimit(&DxGPS, &DyGPS,5.0f,&Dxy)){ //Max 2.5ms/s
//                                        VX_GPS_BODYf+=DxGPS;
//                                        VY_GPS_BODYf+=DyGPS;
//                                      }else {
//                                        VX_GPS_BODYf=VX_GPS_BODY.Out;
//                                        VY_GPS_BODYf=VY_GPS_BODY.Out;
//                                      }
                                     /*                   //-----------Lam mem, giam giat GPS------------
                                      static uint8_t CountGPStime=0;
                                      if(++CountGPStime>=5)CountGPStime=5;
                                      if(CyclicGPS_Updated){
                                        VX_GPS_BODYf=  VX_GPS_BODY_LAST;
                                        VY_GPS_BODYf=  VY_GPS_BODY_LAST;
                                        CyclicGPS_Updated=0;
                                        CountGPStime=0;
                                      }
                                      else {
                                        VX_GPS_BODYf=VX_GPS_BODY_LAST+ (VX_GPS_BODY.Out-VX_GPS_BODY_LAST)*(float)CountGPStime/5.0f;
                                        VY_GPS_BODYf=VY_GPS_BODY_LAST+ (VY_GPS_BODY.Out-VY_GPS_BODY_LAST)*(float)CountGPStime/5.0f;
                                      }
                                      */

                                    if(VX_Accel<VX_GPS_BODY.Out)VX_Accel+=0.2f; else VX_Accel-=0.2f;
                                    if(VY_Accel<VY_GPS_BODY.Out)VY_Accel+=0.2f; else VY_Accel-=0.2f;

                             } // End 50Hz

                             //---------------------------500Hz------------------------
                                  VX_BODY_SET_SOFT+=VY_BODY_SET_SOFT*Z_GyroRate;
                                  VY_BODY_SET_SOFT-=VX_BODY_SET_SOFT*Z_GyroRate;

                                  Kalman_XError+=(Kalman_YError)*Z_GyroRate;   // Quay theo tam offset khac 0
	                              Kalman_YError-=(Kalman_XError)*Z_GyroRate;

                                  I_ANGLE_X+=I_ANGLE_Y*Z_GyroRate;   // Quay theo tam offset khac 0
	                              I_ANGLE_Y-=I_ANGLE_X*Z_GyroRate;


				                  //VX_GPS_BODY.Out+=AX_horizontal*(SCALE_ACCEL/(float)Frq_Update); // cm/s  418*400 =167200<=>1m/s    100/418/500
				                 // VY_GPS_BODY.Out+=AY_horizontal*(SCALE_ACCEL/(float)Frq_Update);

                                  float VXE=VX_GPS_BODY.Out-VX_Accel;
                                  float VYE=VY_GPS_BODY.Out-VY_Accel;

                                  VX_Accel+=VXE/((float)Frq_Update*1.75f);	// Fast Sync IMU Speed
                                  VY_Accel+=VYE/((float)Frq_Update*1.75f);


                                  BrakeX+=(BrakeY)*Z_GyroRate; //Xoay theo Yaw
	                              BrakeY-=(BrakeX)*Z_GyroRate;

                                  BrakeX-=BrakeX/((float)Frq_Update);
                                  BrakeY-=BrakeY/((float)Frq_Update);

                                  float X_DriftNow= (VX_Accel-(VX_BODY_SET_SOFT*1.08f))*3.0f+ (AX_horizontal*(0.30f*SCALE_ACCEL));
                                  float Y_DriftNow= (VY_Accel-(VY_BODY_SET_SOFT*1.08f))*3.0f+ (AY_horizontal*(0.30f*SCALE_ACCEL));

                                  X_ERR_HOVER=((BrakeX+ (Kalman_XError*1.1f))*(float)GPS_PGAIN/50.0f);
		                          Y_ERR_HOVER=((BrakeY+ (Kalman_YError*1.1f))*(float)GPS_PGAIN/50.0f);

  // ------------------------------AutoHover GPS not Move Cyclic------------------------------------------------
                              if((TxIdleTime>=(Frq_Update*12)) && (GPS.TimeoutGPS<Frq_Update) &&  (GPS.HDOP<(VHDOP_HOME*2))) {  //Hold/GoHome/Waypoint
                                 Kalman_XError+=(VX_BODY_SET_SOFT-VX_Accel)/(float)Frq_Update;	 // Sai so vi tri theo tich luy van toc Accel
			                     Kalman_YError+=(VY_BODY_SET_SOFT-VY_Accel)/(float)Frq_Update;

                                 X_DriftHover=X_DriftNow;
                                 Y_DriftHover=Y_DriftNow;

                                 Kalman_XError+=(X_BODY_POS_CM_ERROR-Kalman_XError)/((float)Frq_Update*0.75f); //-(VX_GPS_BODY.Out*0.1f);  // Anti Delay GPS
                                 Kalman_YError+=(Y_BODY_POS_CM_ERROR-Kalman_YError)/((float)Frq_Update*0.75f);//-(VY_GPS_BODY.Out*0.1f);

  //--------------------------Go Home SWitch Waypoints 50Hz Loop - NotMoveTX--------------------
                                 if(CountLoop50Hz==2)
                                 {
                                    // KalmanFiller(&Kalman_XError,PosErrX, 0.003f); //0.03Hz
                                   //  KalmanFiller(&Kalman_YError,PosErrY, 0.003f);
                                     if(FlyMode==GOHOME_MODE){ //Start Going Home
                                    	  int32_t RTH_Lon=GPS.LonHome;
                                    	  int32_t RTH_Lat=GPS.LatHome;
                                    	 if(GPS_Base.HomeFixed && GPS_Base.TimeoutGPS<250&& GPS_Base.HDOP<VHDOP_HOME){
                                    		 RTH_Lon=GPS_Base.Lon;
                                    		 RTH_Lat=GPS_Base.Lat;
                                    	 }
                                         float SpeedUpDownGoHome;
                                         if(GoingHomeStep==1) {	 // Up
			                                  SpeedUpDownGoHome=SPEED_UP_DOWN_GOHOME; //Cm/s
			                                  if(AltitudeBaroCmLP<((float)MinAltiRTH+AltiHome))Alti_CON+=SpeedUpDownGoHome/50.0f;
                                              uint8_t TailOK= SetHeadtoPoint(RTH_Lon,RTH_Lat,0,&AngleFlybar[2]);
			                                  if((DistanceHomeCm<500)||(AltitudeBaroCmLP>=((float)MinAltiRTH+AltiHome) && TailOK))GoingHomeStep=2 ;// Wait Tail and Alti OK over 5m
                                              HoverGPS();
                                              FlyingStatus=GOHOME_UP;
	                                      }
                                          else if(GoingHomeStep==2){//GoingHome
                                               if((GotoOnePoint(RTH_Lon,RTH_Lat,(float)RTH_SPEED_CM,(float)GPS_BRAKE,0)<1.0f)) {
                                            	   HoverGPS();
                                            	   if((LON_GPS_ERROR>-100&& LON_GPS_ERROR<100 &&LAT_GPS_ERROR>-100 &&LAT_GPS_ERROR<100))GoingHomeStep=3;
                                               } else SetHeadtoPoint(RTH_Lon,RTH_Lat,0,&AngleFlybar[2]);
                                                  FlyingStatus=GOHOME_MOVING;
                                               //   if(DistanceFM24Near<200 && TimOutFM24NearVail<300)Alti_CON+=1.0f; // Go Up When not Point
                                          }
                                          else if(GoingHomeStep==3){ // go Down
                                                    GPS.LonSet=(double)RTH_Lon; GPS.LatSet=(double)RTH_Lat;
                                                    SpeedUpDownGoHome=(AltitudeBaroCmLP-AltiHome)/10.0f; 	          // SLow down on Ground, drift 3m baro offset
				                                    if(SpeedUpDownGoHome<40)SpeedUpDownGoHome=40;	 // Xuong cham dan
                                                    else if(SpeedUpDownGoHome>SPEED_UP_DOWN_GOHOME)SpeedUpDownGoHome=SPEED_UP_DOWN_GOHOME;
                                                  //  if((LON_GPS_ERROR>-100&& LON_GPS_ERROR<100 &&LAT_GPS_ERROR>-100 &&LAT_GPS_ERROR<100) ||(AltitudeBaroCmLP>=(2000+AltiHome)))
                                                    Alti_CON-=SpeedUpDownGoHome/50.0f; //30m

//                                                    if(RTH_TAIL_MODE==1) SetHeading(TakeoffAngle, &AngleFlybar[2]);
                                                    HoverGPS();
                                                    FlyingStatus=GOHOME_DOWN;
                                          }
                                      }
/*                                      else if(FlyMode==WAYPOINT_MODE && (WayPoints.NumberPoints>=2)){
                                           if(WayPointsFlying(&WayPoints)){
                                               FlyingStatus=WAYPOINT_MOVING;
                                               if(FM24_AltiMode &&TimOutFM24NearVail<1500) { // Enable Radar for Spraying
                                                if(TimOutFM24NearVail<1500 && DistanceFM24Near<80 && DistanceFM24Near>10 && DistanceFM24Near<LIMIT_RADAR_CM) { //Too Low level <80cm
                                                   Alti_CON+=0.5f;
                                                }
                                                else if(FM24_AltiMode==1 && TimeAltiNotMoveTx && TimOutFM24NearVail<1500 && DistanceFM24Near<LIMIT_RADAR_CM ){// TreeAlti Hold
                                                 if(((uint16_t)(DistanceFM24Near+RADAR_NOISE)<FM24_AltiSet) || (DistanceFM24Ground<100)){
                                                    Alti_CON+=0.2f;//+(FM24_AltiSet-DistanceFM24Near)*0.006f; //15cm/s
                                                 }  else if(DistanceFM24Near>(FM24_AltiSet+RADAR_NOISE)){
                                                    Alti_CON-=0.2f;//10cm/s
                                                 }
                                               }
                                               else if(FM24_AltiMode==2&& TimeAltiNotMoveTx&& TimOutFM24NearVail<1500&& DistanceFM24Near<LIMIT_RADAR_CM){  // Ground Alti Hold
                                                 if((uint16_t)(DistanceFM24Ground+RADAR_NOISE)<FM24_AltiSet){
                                                    Alti_CON+=0.2f;//+(FM24_AltiSet-DistanceFM24Ground)*0.006f; ; //15cm/s
                                                 }  else if(DistanceFM24Ground>(FM24_AltiSet+RADAR_NOISE)){
                                                    Alti_CON-=0.2f;
                                                 }
                                                }
                                               }
                                               else  {  // Waypoint no Radar or Lost Radar
                                                 if((GPS.AltitudeCm-AltiHomeGPS)<(WayPoints.Altitude[WayPoints.PointFlying]-10)) Alti_CON+=0.2f;
                                                 else if((GPS.AltitudeCm-AltiHomeGPS)>(WayPoints.Altitude[WayPoints.PointFlying]+10))Alti_CON-=0.2f;
                                               }

                                           } else {  // Wayponit Done
                                                FlyingStatus=WAYPOINT_DONE;
                                                HoverGPS();
//                                                if(TimeAltiNotMoveTx&& TimOutFM24NearVail<250&& DistanceFM24Near<500){// Test GPS hold Radar
//                                                 if(((DistanceFM24Near+RADAR_NOISE)<FM24_AltiSet) || (DistanceFM24Ground<100)){
//                                                     Alti_CON+=0.6f; //30cm/s
//                                                 }  else if(DistanceFM24Near>(FM24_AltiSet+RADAR_NOISE)){
//                                                     Alti_CON-=0.5f;//25cm/s
//                                                 }
//                                               }
                                           }
                                           //if(WayPointsFlying(&WayPoints[0],0,2)==0);//FlyMode=GPS_HOLD_MODE; // WaypointDone
                                            GoingHomeStep=1;
                                         }*/
                                         else { // GPS HOld
                                            GoingHomeStep=1; FlyingStatus=GPS_HOVERING;
                                            HoverGPS();
                                         }

                                        //----------------Hover Alti Radar---------------------
                                        if(TimeAltiNotMoveTx>0 &&(FlyingStatus==GPS_HOVERING ||FlyingStatus==WAYPOINT_DONE)){
#ifdef RADAR_ALTI_HOVER
                                            if( TimOutFM24NearVail<250 && DistanceFM24Ground>20 && DistanceFM24Ground<LIMIT_RADAR_CM){
                                               if(++CountHoverVailRadar<=150){ //3s
                                                   AltiRadarHover=DistanceFM24Near;
                                               } else {
                                                   if((DistanceFM24Near+RADAR_NOISE)<AltiRadarHover)Alti_CON+=0.2f; //10cm/s
                                                   else if((DistanceFM24Near-RADAR_NOISE)>AltiRadarHover)Alti_CON-=0.2f;
                                               }
                                            } else CountHoverVailRadar=0;
#endif

                                           //-----------------------Low Battery Calu by Throttle---------------------------
                                            #ifdef LOWBAT_BY_THROTTLE_LEVEL
                                            if((TimeAltiNotMoveTx>(Frq_Update*3))&&(countTimeBaroOnning>(Frq_Update*3))&&TxMoveDelay[2]==0){ ///LowBat
                                              static float ThrottleHoverFrist=0;
                                              ThroHover+=((float)PIT_CON-ThroHover)*0.005f;
                                              if((float)PIT_CON>ThroHover)ThroHover+=1.0f; else ThroHover-=1.0f;

                                              if(TimeAltiNotMoveTx>(Frq_Update*10) && ThrottleHoverFrist==0)ThrottleHoverFrist=ThroHover;
                                              if(ThrottleHoverFrist>5500){
                                                float PercentBatf=100- (ThroHover-ThrottleHoverFrist)/6.4f;
                                                if(PercentBatf<0)PercentBatf=0;
                                                PercentBat=(uint16_t)PercentBatf;
                                              }
                                            }
                                            #endif
                                        }
                                        else CountHoverVailRadar=0;
                                     // ----END Hover Alti Radar-----

                                         if(TakeOffOK && VBODYControl==0 &&(FlyingStatus==GPS_HOVERING ||FlyingStatus==WAYPOINT_DONE || FlyingStatus==GOHOME_DOWN)){  // Chi lay I  hover khi hover GPS
                                            if(TxMoveDelay[2]==0&& GPS.HDOP>500 &&TxIdleTime>=(Frq_Update*16)){ // Khu Drift Noise GPS
                                             static int32_t LON_GPS_ERROR_TRUE=0,LAT_GPS_ERROR_TRUE=0;
                                             float XYE;
                                             if(RingLimit(&VXE, &VYE,150.0f,&XYE)){
                                                 GPS.LonSet=(double)((int32_t)LON_GPS_ERROR_TRUE+GPS.Lon);
                                                 GPS.LatSet=(double)((int32_t)LAT_GPS_ERROR_TRUE+GPS.Lat);
                                                 VX_GPS_BODY.Out=VX_Accel+VXE;
                                                 VY_GPS_BODY.Out=VY_Accel+VYE;
                                                 //TIM8->CCR1 = 140;  // Err LED
                                             }
                                             else {
                                               LON_GPS_ERROR_TRUE=LON_GPS_ERROR;
                                               LAT_GPS_ERROR_TRUE=LAT_GPS_ERROR;
                                             }
                                            }

                                            float EX=X_BODY_POS_CM_ERROR;
                                            float EY=Y_BODY_POS_CM_ERROR;
                                            float EXY;
                                            if(RingLimit(&EX, &EY,POS_ERROR,&EXY)){ //Err over 20Cm
//                                                I_ANGLE_X+=(X_BODY_POS_CM_ERROR)/300.0f;  //  Keo Drone ve Center cham
//                                                I_ANGLE_Y+=(Y_BODY_POS_CM_ERROR)/300.0f;
                                              I_ANGLE_X+=(X_BODY_POS_CM_ERROR-EX)/250.0f;  //  Keo Drone ve Center cham
                                              I_ANGLE_Y+=(Y_BODY_POS_CM_ERROR-EY)/250.0f;

                                              float RingVal;
                                              RingLimit(&I_ANGLE_X, &I_ANGLE_Y,300.0f,&RingVal);
                                            }
                                              //---------------Hover Angle -----------------
                                         //   HoverAngle[0]+=(Angle[0]-HoverAngle[0])/300.0f;
                                          //  HoverAngle[1]+=(Angle[1]-HoverAngle[1])/300.0f;
                                         }
                                         else { I_ANGLE_X-=I_ANGLE_X*0.005f; I_ANGLE_Y-=I_ANGLE_Y*0.005f; }
                               }// End 50Hz
                            }
                            else { // ---------------------------------Moving Cyclic Tx------------------------------------------
                                 FlyingStatus=GPS_MOVING;
                                 GoingHomeStep=1;
                                 VX_GPS_SET_LAST_POINT=0;
                                 VY_GPS_SET_LAST_POINT=0;

                                 VX_Accel+=VXE/((float)Frq_Update*1.2f);
                                 VY_Accel+=VYE/((float)Frq_Update*1.2f);

                                 if(TxIdleTime<(Frq_Update))
                                 {
                                      X_DriftNow=X_DriftHover+((X_DriftNow-X_DriftHover)*(float)(TxIdleTime+100))/((float)(Frq_Update+100));
                                      Y_DriftNow=Y_DriftHover+((Y_DriftNow-Y_DriftHover)*(float)(TxIdleTime+100))/((float)(Frq_Update+100));
                                      if(VGPS<30 && VBODYControl<20) TxIdleTime++;

                                      //----------------Return Zero-------------------
                                      if(CountLoop50Hz==4){
                                        Kalman_XError-=Kalman_XError/((float)Frq_Update*1.5f);
                                        Kalman_YError-=Kalman_YError/((float)Frq_Update*1.5f);
                                        LON_GPS_ERROR_HOVER-=LON_GPS_ERROR_HOVER/((float)Frq_Update*1.5f);
                                        LAT_GPS_ERROR_HOVER-=LAT_GPS_ERROR_HOVER/((float)Frq_Update*1.5f) ;
                                        X_DriftHover-=X_DriftHover/((float)Frq_Update*1.5f);
                                        Y_DriftHover-=Y_DriftHover/((float)Frq_Update*1.5f);
                                        I_ANGLE_X-=I_ANGLE_X/((float)Frq_Update*1.5f);
                                        I_ANGLE_Y-=I_ANGLE_Y/((float)Frq_Update*1.5f);
                                      }

                                 } else {//over 1s no Stick Move
                                     if(XY_DriftRing>=275.0f)  {
                                        VX_Accel+=VXE/((float)Frq_Update*1.0f);
                                        VY_Accel+=VYE/((float)Frq_Update*1.0f);

                                        if(VBODYControl>50){
                                            float KAcellTx=VGPS/VBODYControl/1.08f;
                                            float VxConNew=VX_BODY_SET_SOFT*KAcellTx; // Xoay theo huong Vector lai Tx
                                            float VyConNew=VY_BODY_SET_SOFT*KAcellTx;

                                            VX_BODY_SET_SOFT+=(VxConNew-VX_BODY_SET_SOFT)/((float)Frq_Update/4.0f);	//Keo Vantoc Tx theo GPS
                                            VY_BODY_SET_SOFT+=(VyConNew-VY_BODY_SET_SOFT)/((float)Frq_Update/4.0f);
                                        }
                                      }
                                      else if(VBODYControl==0){
                                             if(VGPS<15){
                                               TxIdleTime+=3;
                                               if(TxIdleTime<(Frq_Update*10)) TxIdleTime=(Frq_Update*10);
                                             }
                                             else if(VGPS<30) TxIdleTime+=2;
                                             else if(VGPS<50) TxIdleTime++;
                                             if(VGPS<100) {
                                                BrakeX-=VX_GPS_BODY.Out/((float)Frq_Update*4.0f);
                                                BrakeY-=VY_GPS_BODY.Out/((float)Frq_Update*4.0f);
                                             }
                                      }

                                      if(TxIdleTime>=(Frq_Update*10)){
                                                BrakeX-=BrakeX/((float)Frq_Update*0.2f);
                                                BrakeY-=BrakeY/((float)Frq_Update*0.2f);
                                      }
                                }
                                GPS.LonSet=(double)((int32_t)(LON_GPS_ERROR_HOVER) +GPS.Lon); //+ VX_GPS
                                GPS.LatSet=(double)((int32_t)(LAT_GPS_ERROR_HOVER) +GPS.Lat); //+ VY_GPS

                                if(CountLoop50Hz==3){
                                        //----------------------Toc do theo Tx--------------------------
                                        VX_BODY_SET= (float)TxMove2[0]/1.6f;  VY_BODY_SET= (float)TxMove2[1]/1.6f; //10m/s
                                        float Dvx,Dvy,Dvxy;
                                        Dvx=VX_BODY_SET-VX_BODY_SET_SOFT;
                                        Dvy=VY_BODY_SET-VY_BODY_SET_SOFT;

                                        VX_BODY_SET_SOFT+=Dvx*0.0025f;
                                        VY_BODY_SET_SOFT+=Dvy*0.0025f;
                                        if(RingLimit(&Dvx, &Dvy,0.8f,&Dvxy)){  //40cms/s    TxMovingRing
                                           VX_BODY_SET_SOFT+=Dvx;
                                           VY_BODY_SET_SOFT+=Dvy;
                                        } else {
                                          VX_BODY_SET_SOFT=VX_BODY_SET;
                                          VY_BODY_SET_SOFT=VY_BODY_SET;
                                        }

                                      // ------------------Anti Error Compass-----------------------
                                        /*
                                    if(VBODYControl>100 &&VGPS>100 && VXY_IMU>100 && GPS.HDOP<(VHDOP_HOME-500)){
                                       float AngleCompas=HeadVGPS-HeadVControl;    //HeadVBODY
                                       float ErrCompass=AngleCompas-angle_HMC;
                                       if(ErrCompass>Pi)ErrCompass-=2*Pi; else if(ErrCompass<-Pi)ErrCompass+=2*Pi;
                                       if((ErrCompass<(Pi*0.33f)) && (ErrCompass>(Pi*-0.33f))) angle_HMC+=ErrCompass*0.15f;

                                    } // End Anti Erro Compass
                                    */
                                  }
                             }  // End Moving Cyclic

                             RingLimit(&X_DriftNow, &Y_DriftNow,275.0f,&XY_DriftRing);  // Limit Angle 16Deg

                             X_CON_GPS=(X_ERR_HOVER+I_ANGLE_X-X_DriftNow)*(float)GPS_GAIN/2250.0f;
                             Y_CON_GPS=(Y_ERR_HOVER+I_ANGLE_Y-Y_DriftNow)*(float)GPS_GAIN/2250.0f;

//                             X_CON_GPS=KalmanFiller(&Kalman_XCON_GPS,X_CON_GPS, 40.0f/(float)Frq_Update);
//                             Y_CON_GPS=KalmanFiller(&Kalman_YCON_GPS,Y_CON_GPS, 40.0f/(float)Frq_Update);

                             float tmpf;
                             RingLimit(&X_CON_GPS, &Y_CON_GPS,20.0f,&tmpf);
}

void GPS_CyclicModeStoping(void){
             // beta=0.006f;
	          VX_Accel-=VX_Accel*0.0001f;
	          VY_Accel-=VY_Accel*0.0001f;

	          KalmanDiftX.Out=0;  KalmanDiftY.Out=0;
              CountGPS_ModeTime=0;
              Kalman_XError=0;
              Kalman_YError=0;
              BrakeX=0;  BrakeY=0;
              GPS.LonSet=(double)GPS.Lon;
              GPS.LatSet=(double)GPS.Lat;

              VX_BODY_SET=0; VY_BODY_SET=0;
              VX_BODY_SET_SOFT=0; VY_BODY_SET_SOFT=0;
              LON_GPS_ERROR_HOVER=0; LAT_GPS_ERROR_HOVER=0;
              X_BODY_POS_CM_ERROR=0;
              Y_BODY_POS_CM_ERROR=0;
               WayPoints.PointFlying=0;
               I_ANGLE_X-=I_ANGLE_X*0.01f;  I_ANGLE_Y-=I_ANGLE_Y*0.01f;
               X_CON_GPS-=X_CON_GPS*0.01f;  Y_CON_GPS-=Y_CON_GPS*0.01f;
               FlyingStatus=NOT_GPS_NOT_ALTI;

               GoingHomeStep=1;
}

void GPS_CyclicAngleMode(void){
              //beta=0.006f;
              CountGPS_ModeTime=0;
	          VX_Accel-=VX_Accel*0.0001f;
	          VY_Accel-=VY_Accel*0.0001f;
              KalmanDiftX.Out=0;  KalmanDiftY.Out=0;
              Kalman_XError=0;
              Kalman_YError=0;
              BrakeX=0;  BrakeY=0;
              GPS.LonSet=(double)GPS.Lon;
              GPS.LatSet=(double)GPS.Lat;
              LONGTI_OLD=GPS.Lon;
	          LATI_OLD=GPS.Lat;
              WayPoints.PointFlying=0;
              VX_BODY_SET=0;
              VY_BODY_SET=0;
              VX_BODY_SET_SOFT=0; VY_BODY_SET_SOFT=0;
              LON_GPS_ERROR_HOVER=0;LAT_GPS_ERROR_HOVER=0;
              X_BODY_POS_CM_ERROR=0;
              Y_BODY_POS_CM_ERROR=0;

               I_ANGLE_X-=I_ANGLE_X*0.01f;  I_ANGLE_Y-=I_ANGLE_Y*0.01f;
               X_CON_GPS-=X_CON_GPS*0.01f;  Y_CON_GPS-=Y_CON_GPS*0.01f;
               FlyingStatus=NOT_GPS;
               GoingHomeStep=1;
}


void AutoPilot(void){ // 400Hz

 }
