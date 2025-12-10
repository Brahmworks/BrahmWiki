#include "servo_control.h"
#include "cJSON.h"
#include <math.h>

ServoControl::ServoControl() {
    _serial = NULL;
}

void ServoControl::begin(HardwareSerial* serial, int rx_pin, int tx_pin) {
    _serial = serial;
    // Initialize serial with 1Mbps, 8N1, on specified pins
    _serial->begin(1000000, SERIAL_8N1, rx_pin, tx_pin);
    st.pSerial = _serial;
}

void ServoControl::moveServo(int servo_id, int angle, int speed, int accel) {
    // Map degrees to servo position scale (0-4095)
    // 0 -> 0
    // 360 -> 4095
    int pos = (int)round((angle / 360.0) * 4095);
    
    // Safety clamping
    if (pos > 4095) pos = 4095;
    if (pos < 0) pos = 0;

    st.WritePosEx(servo_id, pos, speed, accel);
}

bool ServoControl::processCommand(const char* json_command, int servo_id) {
    cJSON *root = cJSON_Parse(json_command);
    if (root == NULL) {
        return false; // Invalid JSON
    }

    cJSON *angleItem = cJSON_GetObjectItem(root, "angle");
    cJSON *speedItem = cJSON_GetObjectItem(root, "speed");
    cJSON *accelItem = cJSON_GetObjectItem(root, "accel");

    // Default values if not specified
    int angle = 0;
    int speed = 1000;
    int accel = 20;

    if (cJSON_IsNumber(angleItem)) {
        angle = angleItem->valueint;
    }
    
    if (cJSON_IsNumber(speedItem)) {
        speed = speedItem->valueint;
    }

    if (cJSON_IsNumber(accelItem)) {
        accel = accelItem->valueint;
    }

    cJSON_Delete(root);

    moveServo(servo_id, angle, speed, accel);
    return true;
}
