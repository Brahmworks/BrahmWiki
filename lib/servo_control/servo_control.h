#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <SCServo.h>

// Servo IDs
#define SERVO_ID_TORSO 1
#define SERVO_ID_NECK  2
#define SERVO_ID_HEAD  3

class ServoControl {
public:
    ServoControl();
    
    /**
     * @brief Initialize the servo communication
     * @param serial Pointer to HardwareSerial instance
     * @param rx_pin RX Pin number
     * @param tx_pin TX Pin number
     */
    void begin(HardwareSerial* serial, int rx_pin, int tx_pin);

    /**
     * @brief Process a JSON command string and move the specified servo
     * @param json_command JSON string e.g. {"angle":180,"speed":600,"accel":50}
     * @param servo_id ID of the servo to move (optional, if logic requires specific ID handling)
     * @return true if command parsed and sent successfully
     */
    bool processCommand(const char* json_command, int servo_id);

    /**
     * @brief Low-level move command
     * @param servo_id ID of the servo
     * @param angle Angle in degrees (0-360)
     * @param speed Speed (default 1000)
     * @param accel Acceleration (default 20)
     */
    void moveServo(int servo_id, int angle, int speed, int accel);

private:
    SMS_STS st;
    HardwareSerial* _serial;
};

#endif // SERVO_CONTROL_H
