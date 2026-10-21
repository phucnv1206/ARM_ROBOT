#include "as5600.h"

void asInit(){
    Wire.begin(SDA, SCL);
    Wire.setClock(800000);
    Wire.setTimeOut(2);  // ms: tránh chặn quá lâu nhịp điều khiển 400 Hz
}

uint16_t readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_REG);

    if (Wire.endTransmission(false) != 0) {
        return 0xFFFF;
    }

    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
        return 0xFFFF;
    }

    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();

    return ((highByte << 8) | lowByte) & 0x0FFF;
}

float AngleDeg(){
    uint16_t raw = readRawAngle();
    if (raw != 0xFFFF) {
      return raw * 360.0f / 4096.0f;
    }else{
        return -1.0;
    }
}
