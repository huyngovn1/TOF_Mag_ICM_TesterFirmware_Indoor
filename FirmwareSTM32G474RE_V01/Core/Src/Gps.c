/*
 * Gps.c
 *
 *  Created on: Oct 1, 2026
 *      Author: HP
 */
#include <Gps.h>
#include "Global.h"

const uint8_t PRT_Config[28] = {  //115200 baud
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xB8, 0x42
};

const uint8_t PRT_Config_RTK[28] = {  // 115200 baud + UBX+NMEA+RTCM3 in
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xDA, 0x52  // Checksum mới đã khớp với u-center
};

const uint8_t RATE_5Hz_Config[14] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00, 0x01, 0x00, 0x01, 0x00,
    0xDE, 0x6A  // Checksum chuẩn do u-center xuất ra
};

const uint8_t MSG_NAV5_Airborne1g[44] = {
		0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
		    0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
		    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
		    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
		    0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x4C, 0xBA
};

const uint8_t RMX_Config[10]={0xB5 ,0x62 ,0x06 ,0x11 ,0x02 ,0x00 ,0x08 ,0x00 ,0x21 ,0x91};
const uint8_t PMS_Config[16]={0xB5 ,0x62 ,0x06 ,0x86 ,0x08 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x94 ,0x5A };
const uint8_t SAVE_Config[21] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0x21, 0xAF
};

const uint8_t MSG_DISABLE_POSLLH[16] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x12, 0xB9
};

//const uint8_t MSG_Config[16]={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0x01 ,0x02 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x13 ,0xBE};
// UBX-CFG-MSG: Cấu hình tần suất bản tin (Ví dụ: bật UBX-NAV-PVT qua UART)
const uint8_t MSG_Config[16] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x18, 0xE1
};
const uint8_t RATE_10Hz_Config[14]={ 0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0x64 ,0x00 ,0x01 ,0x00 ,0x01 ,0x00 ,0x7A ,0x12}; //10Hz
const uint8_t PWR_Config[16] ={0xB5 ,0x62 ,0x06 ,0x57 ,0x08 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0x20 ,0x4E ,0x55 ,0x52 ,0x7B ,0xC3 };
const uint8_t GNSS_Config[60] = {
		    0xB5, 0x62, 0x06, 0x3E, 0x34, 0x00, 0x00, 0x00,
		    0x31, 0x06, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
		    0x01, 0x01, 0x01, 0x03, 0x03, 0x00, 0x01, 0x00,
		    0x01, 0x01, 0x02, 0x08, 0x0C, 0x00, 0x01, 0x00,
		    0x01, 0x01, 0x03, 0x06, 0x0E, 0x00, 0x01, 0x00,
		    0x01, 0x01, 0x05, 0x03, 0x04, 0x00, 0x01, 0x00,
		    0x05, 0x01, 0x06, 0x08, 0x0C, 0x00, 0x00, 0x00,
		    0x00, 0x01, 0x35, 0x78
};

const uint8_t MSG_SAVE_CFG[21] = {
    0xB5, 0x62,                         // Sync Header
    0x06, 0x09,                         // Class: CFG (0x06), ID: CFG (0x09)
    0x0D, 0x00,                         // Length: 13 Bytes (0x000D)
    0x00, 0x00, 0x00, 0x00,             // clearMask: 0x00000000
    0xFF, 0xFF, 0x00, 0x00,             // saveMask: 0x0000FFFF (Save all sections)
    0x00, 0x00, 0x00, 0x00,             // loadMask: 0x00000000
    0x03,                               // deviceMask: 0x03 (0 - BBR + 1 - FLASH)
    0x1D, 0xAB                          // Checksum CK_A, CK_B
};


void M10N_GPS_Config(UART_HandleTypeDef *huart) {
    // -------------------------------------------------------------
    // BƯỚC 1: Khởi tạo UART STM32 ở 38400 bps (Default của M10)
    // -------------------------------------------------------------
    HAL_UART_DeInit(huart);
    huart->Init.BaudRate = 9600;
    HAL_UART_Init(huart);
    HAL_Delay(100);

    // -------------------------------------------------------------
    // BƯỚC 2: Gửi lệnh đổi Baudrate M10N
    // -------------------------------------------------------------
    HAL_UART_Transmit(huart, (uint8_t *)PRT_Config, 28, 100);
    HAL_Delay(100); // Chờ M10 đổi tốc độ

    // -------------------------------------------------------------
    // BƯỚC 3: Đổi UART STM32 lên 115200 bps để đồng bộ
    // -------------------------------------------------------------
    HAL_UART_DeInit(huart);
    huart->Init.BaudRate = 115200;
    HAL_UART_Init(huart);
    HAL_Delay(100);

    // Cấu hình Năng lượng / Power Mode
    HAL_UART_Transmit(huart, (uint8_t *)PWR_Config, 16, 100);
    HAL_Delay(100);

    // -------------------------------------------------------------
    // BƯỚC 4: Gửi tất cả các cấu hình hoạt động ở tốc độ 115200 bps
    // -------------------------------------------------------------

    HAL_UART_Transmit(huart, (uint8_t *)PMS_Config, sizeof(PMS_Config), 100);  // Full Power
    HAL_Delay(100);

    HAL_UART_Transmit(huart, (uint8_t *)SAVE_Config, sizeof(SAVE_Config), 100);
    HAL_Delay(200);

    HAL_UART_Transmit(huart, (uint8_t *)GNSS_Config, sizeof(GNSS_Config), 100);
    HAL_Delay(100);


    // Dynamic Model = Airborne < 1g (Tối ưu Flycam)
    HAL_UART_Transmit(huart, (uint8_t *)MSG_NAV5_Airborne1g, sizeof(MSG_NAV5_Airborne1g), 100);
    HAL_Delay(100);

    HAL_UART_Transmit(huart, (uint8_t *)MSG_Config, 16, 100);
    HAL_Delay(100);

    // Tần số cập nhật 10Hz (100ms)
    HAL_UART_Transmit(huart, (uint8_t *)RATE_10Hz_Config, 14, 100);
    HAL_Delay(100);

    // -------------------------------------------------------------
    // BƯỚC 5: Lưu cấu hình vào Flash/BBR
    HAL_UART_Transmit(huart, (uint8_t *)MSG_SAVE_CFG, sizeof(MSG_SAVE_CFG), 100);
    HAL_Delay(200);
}


/*uint8_t GPS_NAV_POSLLH_Decoder(GPS_t* GPSin, uint8_t *Buf){   //UBX-NAV-PVT (0x01 0x07) FUll
	uint8_t reval=0;
	   if(Buf[2]!=0x01 ||Buf[3]!=0x02){return 0;}
       uint8_t CKA=0,CKB=0;
        for(uint8_t i=2;i<34;i++){
           CKA+=Buf[i];
           CKB+=CKA;
        }

        uint16_t UBXOffset=Buf[4]|(Buf[5]<<8);
        if((CKA!=Buf[34])||(CKB!=Buf[35]) ||UBXOffset!=28) {return 0;}

        uint32_t TimeWeek=Buf[6]|(Buf[7]<<8)|(Buf[8]<<16)|(Buf[9]<<24);
        GPSin->Lon=Buf[10]|(Buf[11]<<8)|(Buf[12]<<16)|(Buf[13]<<24);   // E-7/Dig <=>1.1cm in VN
        GPSin->Lat=Buf[14]|(Buf[15]<<8)|(Buf[16]<<16)|(Buf[17]<<24);   // E-7/Dig <=>1.1cm in VN
        int32_t AltitudeMm= Buf[18]|(Buf[19]<<8)|(Buf[20]<<16)|(Buf[21]<<24);
        GPSin->HDOP=	Buf[26]|(Buf[27]<<8)|(Buf[28]<<16)|(Buf[29]<<24);  //Horizontal Accuracy Estimate [mm]
        GPSin->VDOP=	Buf[30]|(Buf[31]<<8)|(Buf[32]<<16)|(Buf[33]<<24);  //Vertical Accuracy Estimate [mm]
        GPSin->FrameRateMs=TimeWeek-GPSin->TimeWeek;
        if(GPSin->Lon&&GPSin->Lat&&(GPSin->HDOP<=100000)&&(GPSin->VDOP<=100000) && GPSin->AltitudeCm>-20000 && GPSin->AltitudeCm<1000000 && GPSin->FrameRateMs) {

        	GPSin->DLon=GPSin->Lon-GPSin->LonOld;
        	GPSin->DLat=GPSin->Lat-GPSin->LatOld;

        	GPSin->vx=((float)GPSin->DLon)*1.1f*(1000.0f)/(float)GPSin->FrameRateMs; // Convert to Cm/s  400Hz
        	GPSin->vy=((float)GPSin->DLat)*(1.11f*1000.0f)/(float)GPSin->FrameRateMs;

        	GPSin->LonOld=GPSin->Lon;
        	GPSin->LatOld=GPSin->Lat;

            GPSin->vUp=(((float)AltitudeMm*0.1f)-GPSin->AltitudeCm)*1000.0f/(float)GPSin->FrameRateMs;
            GPSin->AltitudeCm=(float)AltitudeMm*0.1f;

        	GPSin->TimeoutGPS=0;
        	reval= 1;
        }
        else {
        	GPSin->vUp=0;
        }

        GPSin->TimeWeek=TimeWeek;
    	return reval;
}*/

void CheckSum(uint8_t *Buf, uint8_t Len, uint8_t *CKA, uint8_t *CKB){
	*CKA=0;
	*CKB=0;
    for(uint8_t i=0;i<Len;i++) {
       *CKA+=Buf[i];
       *CKB+= *CKA;
    }
}

// Hàm giải mã trực tiếp buffer 100 bytes vào cấu trúc UBX_NAV_PVT_t
uint8_t Parse_And_Update_PVT(const uint8_t *uart_buf, UBX_NAV_PVT_t *pvt_dest) {
    // 1. Kiểm tra Header Sync (0xB5 0x62) và Class/ID (0x01 0x07 cho NAV-PVT)
    if (uart_buf[0] != 0xB5 || uart_buf[1] != 0x62 ||
        uart_buf[2] != 0x01 || uart_buf[3] != 0x07) {
        return 0; // Sai header hoặc sai bản tin
    }

    // 2. Kiểm tra độ dài Payload (Phải là 92 bytes = 0x5C, 0x00 ở byte thứ 4 và 5)
    uint16_t payload_len = (uint16_t)(uart_buf[5] << 8 | uart_buf[4]);
    if (payload_len != 92) {
        return 0;
    }

    // 3. Kiểm tra Checksum (Fletcher Checksum từ byte Class đến hết Payload)
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < 6 + payload_len; i++) {
        ck_a += uart_buf[i];
        ck_b += ck_a;
    }

    // So sánh với 2 byte checksum cuối cùng của mảng (index 98 và 99)
    if (ck_a != uart_buf[98] || ck_b != uart_buf[99]) {
        return 0; // Lỗi Checksum
    }

    // 4. Copy trực tiếp toàn bộ 92 bytes payload vào struct UBX_NAV_PVT_t (bắt đầu từ index 6)
    memcpy(pvt_dest, &uart_buf[6], sizeof(UBX_NAV_PVT_t));

    return 1; // Giải mã và cập nhật thành công
}

uint8_t AirLinkEcoder(const GPS_t *gps,uint8_t ID,uint8_t cmd,uint8_t *buf){
	buf[0]=0x98;
	buf[1]=ID;
	buf[2]=cmd;
	buf[3]=gps->Lon&0xFF;
	buf[4]=(gps->Lon>>8)&0xFF;
	buf[5]=(gps->Lon>>16)&0xFF;
	buf[6]=(gps->Lon>>24)&0xFF;
	buf[7]=gps->Lat&0xFF;
	buf[8]=(gps->Lat>>8)&0xFF;
	buf[9]=(gps->Lat>>16)&0xFF;
	buf[10]=(gps->Lat>>24)&0xFF;

	int32_t AltiMm=gps->AltitudeCm*10;
	buf[11]=AltiMm&0xFF;
	buf[12]=(AltiMm>>8)&0xFF;
	buf[13]=(AltiMm>>16)&0xFF;
	buf[14]=(AltiMm>>24)&0xFF;

	buf[15]=gps->Heading&0xFF;
	buf[16]=(gps->Heading>>8)&0xFF;
	CheckSum(&buf[0], 17, &buf[17], &buf[18]);
	return 1;
}

uint8_t AirLinkDecoder(const uint8_t *buf, AirLink_t *gps) {
    // 1. Kiểm tra Header Sync
    if (buf[0] != 0x98) {
        return 0;
    }

    uint8_t ck_a, ck_b;
    CheckSum((uint8_t *)buf, 17, &ck_a, &ck_b);

    if (ck_a != buf[17] || ck_b != buf[18]) {
        return 0; // Lỗi Checksum
    }

    gps->Id = buf[1];
    gps->Cmd = buf[2];

    int32_t Lon = (int32_t)( ((uint32_t)buf[3])       |
                          ((uint32_t)buf[4] << 8)  |
                          ((uint32_t)buf[5] << 16) |
                          ((uint32_t)buf[6] << 24) );

    int32_t Lat = (int32_t)( ((uint32_t)buf[7])       |
                          ((uint32_t)buf[8] << 8)  |
                          ((uint32_t)buf[9] << 16) |
                          ((uint32_t)buf[10] << 24) );


    int32_t altiMm = (int32_t)( ((uint32_t)buf[11])      |
                                ((uint32_t)buf[12] << 8)  |
                                ((uint32_t)buf[13] << 16) |
                                ((uint32_t)buf[14] << 24) );
    gps->AltitudeCm = (float)altiMm*0.1f;

    gps->Heading = (int16_t)( ((uint16_t)buf[15]) |
                          ((uint16_t)buf[16] << 8) );

    if(gps->Timeout<10) return 0;
    gps->VLon=(Lon-gps->Lon)*400/(float)gps->Timeout;
    gps->VLat=(Lat-gps->Lat)*400/(float)gps->Timeout;
    gps->Lon=Lon;
    gps->Lat=Lat;

    gps->FrameRate=gps->Timeout;
    gps->Timeout=0;

    return 1; // Thành công
}

uint8_t CheckEndRtkPacket(uint8_t *Buf){
   return 1;
}
