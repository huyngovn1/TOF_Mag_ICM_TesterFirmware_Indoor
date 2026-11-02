#include "main.h"
#include <stdint.h>
// Địa chỉ thiết bị
// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
#define CRSF_ADDRESS_FLIGHT_CONTROLLER    0xFC
#define CRSF_FRAMETYPE_BATTERY_SENSORS    0x08

#define CRSF_ADDRESS_RX   0xC8 //   0xEA   //      0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16



// Cấu trúc gói tin CRSF GPS Extended (0x06)

// Định nghĩa cấu trúc chính xác theo ảnh image_8c0415.png
typedef struct __attribute__((packed)) {
    uint8_t  addr;             // 0xC8
    uint8_t  size;             // Kích thước = 1 byte Type + 22 bytes Payload = 23 (0x17)
    uint8_t  type;             // 0x06 (GPS Extended)

    // --- Bắt đầu Payload (Đúng thứ tự trong ảnh) ---
    uint8_t  fix_type;         // Current GPS fix quality
    int16_t  n_speed;          // Northward Speed [cm/sec] - Big Endian
    int16_t  e_speed;          // Eastward Speed [cm/sec] - Big Endian
    int16_t  v_speed;          // Vertical (up = positive) Speed [cm/sec] - Big Endian
    int16_t  h_speed_acc;      // Horizontal Speed accuracy cm/sec - Big Endian
    int16_t  track_acc;        // Heading accuracy in degrees scaled with 1e-1 (deg * 10) - Big Endian
    int16_t  alt_ellipsoid;    // Meters Height above GPS Ellipsoid (not MSL) - Big Endian
    int16_t  h_acc;            // horizontal accuracy in cm - Big Endian
    int16_t  v_acc;            // vertical accuracy in cm - Big Endian
    uint8_t  reserved;         // Byte dự phòng (Gán = 0)
    uint8_t  hDOP;             // Horizontal dilution of precision (Dimensionless in nits of 0.1)
    uint8_t  vDOP;             // Vertical dilution of precision (Dimensionless in nits of 0.1)

    uint8_t  crc;              // Checksum CRC8
} crsf_gps_ext_t;



// Struct Battery chuẩn CRSF (12 bytes tổng cộng)
typedef struct __attribute__((packed)) {
    uint8_t addr;       // 0xC8 (Địa chỉ gửi tới Radio)
    uint8_t size;       // 0x09 (Dòng Type + 8 byte Payload)
    uint8_t type;       // 0x08 (Battery Sensor)
    uint16_t voltage;   // Big Endian (0.1V) - Ví dụ: 16.8V gửi 168
    uint16_t current;   // Big Endian (0.1A) - Ví dụ: 10.5A gửi 105
    uint8_t capacity[3];// Big Endian (1mAh)
    uint8_t remaining;  // 0-100 (%)
    uint8_t crc;        // CRC8
} crsf_battery_t;


typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t size;
    uint8_t type;

    uint16_t voltage;
    uint16_t current;

    uint8_t capacity[3];
    uint8_t remaining;
    uint8_t temp;

    uint16_t rpm;

    uint8_t crc;
} crsf_esc_t;

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t size;
    uint8_t type;         // 0x21 (Flight Mode)
    char mode_str[10];    // Chuỗi văn bản trạng thái (tối đa 10 ký tự bao gồm null)
    uint8_t crc;
} crsf_flight_mode_t;

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t size;           // = 7
    uint8_t type;           // 0x0C
    uint8_t source_id;
    uint32_t rpm_val;       // 4 bytes - Big Endian
    uint8_t crc;
} crsf_rpm_t;



typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t size;
    uint8_t type;          // 0x09
    int16_t vertical_spd;  // Big Endian (cm/s)
    uint16_t reserved;     // Thường là 0
    uint8_t crc;
} crsf_vario_t;

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t size;
    uint8_t type;         // 0x02
    int32_t latitude;     // Big Endian (degree / 10,000,000)
    int32_t longitude;    // Big Endian
    uint16_t groundspeed; // Big Endian (km/h / 10)
    uint16_t heading;     // Big Endian (degree / 100)
    uint16_t altitude;    // Big Endian (meters + 1000)
    uint8_t satellites;   // Số vệ tinh
    uint8_t crc;
} crsf_gps_t;

typedef struct __attribute__((packed)) {
    uint8_t addr;         // 0xC8 (FC) hoặc 0xEA (Radio)
    uint8_t size;         // 4 (Type + Payload 2 byte + CRC 1 byte)
    uint8_t type;         // 0x0D
    // Payload (2 bytes)
    uint8_t sensor_id;    // ID cảm biến: 0, 1, 2...
    int8_t  temp_c;       // Nhiệt độ thực tế (Số nguyên có dấu, đơn vị độ C)
    // Kết thúc
    uint8_t crc;
} crsf_temp_t;
uint8_t crsf_crc8(uint8_t *data, uint8_t len) ;
void Crsf_send_flight_mode(UART_HandleTypeDef *huart, const char* mode_text) ;
void Crsf_send_gps(UART_HandleTypeDef *huart, int32_t lat, int32_t lon, uint16_t speed, uint16_t head, int16_t alt, uint8_t sats) ;
void Crsf_send_battery(UART_HandleTypeDef *huart, uint16_t volt_10, uint16_t curr_10, uint32_t mah, uint8_t percent);
void Altitude_VerticalSpeed(UART_HandleTypeDef *huart, int16_t altitude_packed, int8_t   vertical_speed_packed);
void crsf_send_esc(UART_HandleTypeDef *huart,
                   uint16_t volt_10,
                   uint16_t curr_10,
                   uint32_t mah,
                   uint8_t temp,
                   uint32_t rpm_real);
void Crsf_send_temperature(UART_HandleTypeDef *huart, int16_t temp);
void crsf_send_rpm(UART_HandleTypeDef *huart, uint32_t rpm_real);
void decode_crsf_channels(uint8_t *payload, uint16_t *RxVal);
void Crsf_send_gps_extended(UART_HandleTypeDef *huart, uint8_t fix, int16_t n_speed_cms, int16_t e_speed_cms,
                            int16_t v_speed_cms, int16_t h_speed_acc_cms, int16_t track_acc_deg10,
                            int16_t alt_ellipsoid_m, int16_t h_acc_cm, int16_t v_acc_cm,
                            uint8_t hdop_10, uint8_t vdop_10);
