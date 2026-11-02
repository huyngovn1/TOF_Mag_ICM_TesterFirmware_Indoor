#include "crsf.h"
#include "String.h"

// Định nghĩa các hằng số CRSF
#define CRSF_FRAMETYPE_GPS_EXTENDED     0x06
 uint8_t buf[64];

 // https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#crsf-protocol
// Hàm tính CRC8 chuẩn CRSF
uint8_t crsf_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}



void decode_crsf_channels(uint8_t *payload, uint16_t *RxVal) {
    // Payload bắt đầu từ byte ngay sau byte Type (tổng cộng 22 byte)
	uint16_t ChannelValue[16];
    ChannelValue[1]  = (uint16_t)((payload[0]       | payload[1] << 8)                             & 0x07FF);
    ChannelValue[2]  = (uint16_t)((payload[1] >> 3  | payload[2] << 5)                             & 0x07FF);
    ChannelValue[6]  = (uint16_t)((payload[2] >> 6  | payload[3] << 2 | payload[4] << 10)                & 0x07FF);
    ChannelValue[4]  = (uint16_t)((payload[4] >> 1  | payload[5] << 7)                             & 0x07FF);
    ChannelValue[5]  = (uint16_t)((payload[5] >> 4  | payload[6] << 4)                             & 0x07FF);
    ChannelValue[3]  = (uint16_t)((payload[6] >> 7  | payload[7] << 1 | payload[8] << 9)                 & 0x07FF);
    ChannelValue[7]  = (uint16_t)((payload[8] >> 2  | payload[9] << 6)                             & 0x07FF);
    ChannelValue[8]  = (uint16_t)((payload[9] >> 5  | payload[10] << 3)                            & 0x07FF);
    ChannelValue[9]  = (uint16_t)((payload[11]      | payload[12] << 8)                            & 0x07FF);
    ChannelValue[10]  = (uint16_t)((payload[12] >> 3 | payload[13] << 5)                            & 0x07FF);
    ChannelValue[11] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10)             & 0x07FF);
    ChannelValue[12] = (uint16_t)((payload[15] >> 1 | payload[16] << 7)                            & 0x07FF);
    ChannelValue[13] = (uint16_t)((payload[16] >> 4 | payload[17] << 4)                            & 0x07FF);
    ChannelValue[14] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9)             & 0x07FF);
    ChannelValue[15] = (uint16_t)((payload[19] >> 2 | payload[20] << 6)                            & 0x07FF);
    ChannelValue[0] = (uint16_t)((payload[20] >> 5 | payload[21] << 3)                            & 0x07FF);

    for(uint8_t i=0;i<=15;i++)RxVal[i]=ChannelValue[i]*2+4080;
}

void Crsf_send_battery(UART_HandleTypeDef *huart, uint16_t volt_10, uint16_t curr_10, uint32_t mah, uint8_t percent) {
    static crsf_battery_t bat_pkt;

    // 1. Kiểm tra UART rảnh
    if (huart->gState != HAL_UART_STATE_READY) return;

    // 2. Header
    bat_pkt.addr = CRSF_ADDRESS_RX; // Hoặc 0xEA tùy thuộc vào việc bạn gửi cho ai
    bat_pkt.size = sizeof(bat_pkt) - 2; // = 10
    bat_pkt.type = 0x08;

    // 3. Đảo byte (Sử dụng hàm tối ưu của GCC)
    bat_pkt.voltage = __builtin_bswap16(volt_10);
    bat_pkt.current = __builtin_bswap16(curr_10);

    // 4. Xử lý 24-bit mAh
    bat_pkt.capacity[0] = (uint8_t)(mah >> 16);
    bat_pkt.capacity[1] = (uint8_t)(mah >> 8);
    bat_pkt.capacity[2] = (uint8_t)(mah);

    bat_pkt.remaining = percent;

    // 5. Tính CRC (Tính từ byte Type đến hết Remaining = 9 byte)
    bat_pkt.crc = crsf_crc8(&bat_pkt.type, 9);

    // 6. Gửi qua DMA
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&bat_pkt, sizeof(bat_pkt));
}

void Crsf_send_flight_mode(UART_HandleTypeDef *huart, const char* mode_text) {
    static crsf_flight_mode_t pkt;
    if (huart->gState != HAL_UART_STATE_READY) return;

    pkt.addr = CRSF_ADDRESS_RX; // Đồng nhất với ảnh bạn chụp (200 decimal)
    pkt.size = 12;   // Type(1) + Payload(10) + CRC(1) = 12
    pkt.type = 0x21; // ID cho Flight Mode

    // Xóa trắng buffer và copy chuỗi mới vào
    memset(pkt.mode_str, 0, sizeof(pkt.mode_str));
    strncpy(pkt.mode_str, mode_text, 10);

    // Tính CRC cho 11 byte (Type + 10 byte Payload)
    // pkt.size - 1 = 12 - 1 = 11 byte
    pkt.crc = crsf_crc8(&pkt.type, 11);

    // Gửi đúng 14 byte (Addr + Size + Type + Payload + CRC)
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&pkt, 14);
}
/**
 * @brief Gửi dữ liệu Telemetry ESC về bộ thu RX
 * @param v: Điện áp (Volts) - Ví dụ: 16.8
 * @param a: Dòng điện (Amperes) - Ví dụ: 25.4
 * @param mah: Dung lượng đã tiêu thụ (mAh) - Ví dụ: 500
 * @param rpm: Vòng tua động cơ (Vòng/Phút) - Ví dụ: 12000
 */

void crsf_send_rpm(UART_HandleTypeDef *huart, uint32_t rpm_real)
{

    // Kiểm tra trạng thái UART trước khi truyền DMA
    if (huart->gState != HAL_UART_STATE_READY)
        return;

    uint8_t i = 0;

    buf[i++] = CRSF_ADDRESS_RX;    // Destination address CRSF_ADDRESS_RX
    buf[i++] = 0x06;    // Frame Length (Type + Source + 3 bytes RPM + CRC)
    buf[i++] = 0x0C;    // Frame Type
    buf[i++] = 0x00;    // Source ID

    // Chèn dữ liệu RPM (Big Endian)
    // Ép kiểu (uint8_t) để tránh warning "narrowing conversion"
    buf[i++] = (uint8_t)((rpm_real >> 16) & 0xFF);
    buf[i++] = (uint8_t)((rpm_real >> 8)  & 0xFF);
    buf[i++] = (uint8_t)(rpm_real & 0xFF);

    // Tính CRC: bắt đầu từ Type (buf[2]) đến hết dữ liệu (i-2 bytes)
    buf[i] = crsf_crc8(&buf[2], i - 2);
    i++; // Tăng i lần cuối để có tổng độ dài cần gửi

    // Truyền dữ liệu qua DMA
    HAL_UART_Transmit_DMA(huart, buf, i);
}

/**
 * @brief Gửi nhiệt độ theo chuẩn TBS Spec (Type 0x0D)
 * @param sensor_id: 0x00, 0x01, 0x02...
 * @param temperature: Giá trị độ C (int8_t: -128 đến 127)
 */
void Crsf_send_temperature(UART_HandleTypeDef *huart, int16_t temp) {
    // Kiểm tra trạng thái UART trước khi truyền DMA
    if (huart->gState != HAL_UART_STATE_READY)
        return;

    uint8_t i = 0;

    buf[i++] = CRSF_ADDRESS_RX;    // Destination address
    buf[i++] = 0x05;    // Frame Length (Type + Source + 3 bytes RPM + CRC)
    buf[i++] = 0x0D;    // Frame Type

    buf[i++] = 0x00;    // Source ID
    buf[i++] = (uint8_t)((temp >> 8)  & 0xFF);
    buf[i++] = (uint8_t)(temp & 0xFF);

    // Tính CRC: bắt đầu từ Type (buf[2]) đến hết dữ liệu (i-2 bytes)
    buf[i] = crsf_crc8(&buf[2], i - 2);
    i++; // Tăng i lần cuối để có tổng độ dài cần gửi

    // Truyền dữ liệu qua DMA
    HAL_UART_Transmit_DMA(huart, buf, i);
}


void Crsf_send_gps(UART_HandleTypeDef *huart, int32_t lat, int32_t lon, uint16_t speed, uint16_t head, int16_t alt, uint8_t sats) {
    static crsf_gps_t pkt;
    if (huart->gState != HAL_UART_STATE_READY) return;
    pkt.addr = CRSF_ADDRESS_RX;
    pkt.size = 17;
    pkt.type = 0x02;

    // Đảo byte cho các số lớn
    pkt.latitude = __REV(lat);
    pkt.longitude = __REV(lon);
    pkt.groundspeed = __REV16(speed);
    pkt.heading = __REV16(head);
    pkt.altitude = __REV16(alt + 1000); // Lưu ý cộng 1000 theo spec
    pkt.satellites = sats;

    // Tính CRC cho 16 byte (Type + 15 byte Payload)
    pkt.crc = crsf_crc8(&pkt.type, 16);

    HAL_UART_Transmit_DMA(huart, (uint8_t*)&pkt, 19);
}

void Crsf_send_gps_extended(UART_HandleTypeDef *huart, uint8_t fix, int16_t n_speed_cms, int16_t e_speed_cms,
                            int16_t v_speed_cms, int16_t h_speed_acc_cms, int16_t track_acc_deg10,
                            int16_t alt_ellipsoid_m, int16_t h_acc_cm, int16_t v_acc_cm,
                            uint8_t hdop_10, uint8_t vdop_10)
{
    static crsf_gps_ext_t gps_pkt;

    // 1. Kiểm tra UART rảnh
    if (huart->gState != HAL_UART_STATE_READY) return;

    // 2. Cài đặt Header
    gps_pkt.addr = 0xC8;
    gps_pkt.size = 22;
    gps_pkt.type = 0x06;

    // 3. Gán các biến 1 byte trực tiếp
    gps_pkt.fix_type = fix;
    gps_pkt.reserved = 0x00;
    gps_pkt.hDOP     = hdop_10;
    gps_pkt.vDOP     = vdop_10;

    // 4. Đảo byte sang Big-Endian cho các biến số thực 16-bit (Dùng mã máy GCC tối ưu)
    gps_pkt.n_speed       = (int16_t)__builtin_bswap16((uint16_t)n_speed_cms);
    gps_pkt.e_speed       = (int16_t)__builtin_bswap16((uint16_t)e_speed_cms);
    gps_pkt.v_speed       = (int16_t)__builtin_bswap16((uint16_t)v_speed_cms);
    gps_pkt.h_speed_acc   = (int16_t)__builtin_bswap16((uint16_t)h_speed_acc_cms);
    gps_pkt.track_acc     = (int16_t)__builtin_bswap16((uint16_t)track_acc_deg10);
    gps_pkt.alt_ellipsoid = (int16_t)__builtin_bswap16((uint16_t)alt_ellipsoid_m);
    gps_pkt.h_acc         = (int16_t)__builtin_bswap16((uint16_t)h_acc_cm);
    gps_pkt.v_acc         = (int16_t)__builtin_bswap16((uint16_t)v_acc_cm);

    // 5. Tính toán CRC8 (Tính từ byte Type đến hết byte vDOP, tổng cộng 23 bytes)
    gps_pkt.crc = crsf_crc8(&gps_pkt.type, 21);

    // 6. Đẩy thẳng ra DMA (Non-blocking hoàn toàn)
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&gps_pkt, 24);
}


void Altitude_VerticalSpeed(UART_HandleTypeDef *huart, int16_t altitude_packed, int8_t   vertical_speed_packed) {
    // Kiểm tra trạng thái UART trước khi truyền DMA
    if (huart->gState != HAL_UART_STATE_READY)
        return;

    uint8_t i = 0;

    buf[i++] = CRSF_ADDRESS_RX;    // Destination address
    buf[i++] = 0x05;    // Frame Length (Type + Source + 3 bytes RPM + CRC)
    buf[i++] = 0x09;    // Frame Type

    altitude_packed+=10000;
    buf[i++] = (uint8_t)((altitude_packed >> 8)  & 0xFF);
    buf[i++] = (uint8_t)(altitude_packed & 0xFF);

    buf[i++] = vertical_speed_packed;

    // Tính CRC: bắt đầu từ Type (buf[2]) đến hết dữ liệu (i-2 bytes)
    buf[i] = crsf_crc8(&buf[2], i - 2);
    i++; // Tăng i lần cuối để có tổng độ dài cần gửi

    // Truyền dữ liệu qua DMA
    HAL_UART_Transmit_DMA(huart, buf, i);
}


/**
 * @brief Gửi dữ liệu ESC (bao gồm nhiệt độ) từ FC sang Rx
 * @param temp: Nhiệt độ (độ C) - Ví dụ: 45
 */
/**
 * @brief Gửi dữ liệu ESC Sensor (Type 0x14) theo chuẩn TBS Spec
 * @param volt_10: Điện áp x10 (168 = 16.8V)
 * @param curr_10: Dòng điện x10 (254 = 25.4A)
 * @param mah: Dung lượng tiêu thụ (mAh)
 * @param temp: Nhiệt độ thực tế (độ C)
 * @param rpm_real: Vòng tua thực tế (vòng/phút)
 */
void crsf_send_esc(UART_HandleTypeDef *huart,
                   uint16_t volt_10,
                   uint16_t curr_10,
                   uint32_t mah,
                   uint8_t temp,
                   uint32_t rpm_real)
{
    static crsf_esc_t pkt;

    if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
        return;

    pkt.addr = CRSF_ADDRESS_RX;
    pkt.type = 0x14;

    pkt.voltage = __REV16(volt_10);
    pkt.current = __REV16(curr_10);

    pkt.capacity[0] = (mah >> 16);
    pkt.capacity[1] = (mah >> 8);
    pkt.capacity[2] = (mah);

    pkt.remaining = 0;
    pkt.temp = temp;

    uint16_t rpm = rpm_real / 100;   // hoặc /10
    pkt.rpm = __REV16(rpm);

    pkt.size = 12;

    pkt.crc = crsf_crc8(&pkt.type, 11);

    HAL_UART_Transmit_DMA(huart, (uint8_t*)&pkt, sizeof(pkt));
}

