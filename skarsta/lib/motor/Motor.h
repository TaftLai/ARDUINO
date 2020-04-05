#ifndef ARDUINO_PROJECTS_ROOT_MOTOR_H
#define ARDUINO_PROJECTS_ROOT_MOTOR_H

#include <Arduino.h>
#include <Service.h>

#ifndef __ULTRASONIC__
#include <Rotary.h>
#else
#include <Ultrasonic.h>
#endif

typedef enum {
    CCW,
    CW,
    OFF
} MotorState;

typedef enum {
    UNCALIBRATED,   // nothing calibrated
    SEMICALIBRATED, // bottom calibrated
    CALIBRATED      // bottom-top calibrated
} MotorMode;

#ifdef __EEPROM__
#define ADDRESS_POSITION 0
#define ADDRESS_END_STOP_0 (ADDRESS_POSITION + sizeof(unsigned int))
#define ADDRESS_END_STOP_1 (ADDRESS_END_STOP_0 + sizeof(unsigned int))
#define ADDRESS_MODE (ADDRESS_END_STOP_1 + sizeof(MotorMode))
#endif

#define STOP_POS_DIFF 1
#define MINIMUM_POS_CHANGE 8

class Motor : Service {
private:
#ifndef __ULTRASONIC__
    Rotary sensor;
#else
    Ultrasonic sensor;
#endif
    const uint8_t sensor_pin_1 = 0, sensor_pin_2 = 0;

    bool disabled = false;
    long next_position = -1;
    unsigned int end_stop[2] = {~0u};

    MotorState state = OFF;
    MotorMode mode = UNCALIBRATED;
#ifndef __ULTRASONIC__
    volatile unsigned int position = 0u, position_change = 0u;
#else
    unsigned int position_change = 0u;
#endif

protected:
#ifndef __ULTRASONIC__

    void update_position(unsigned char result);

#endif

    void initPin(uint8_t pin, uint8_t val = LOW);

    virtual void _off() = 0;

    virtual void _dir_cw() = 0;

    virtual void _dir_ccw() = 0;

public:
    Motor(uint8_t _pin1, uint8_t _pin2);

    void begin() override;

    void off();

    void dir_cw();

    void dir_ccw();

    void set_end_stop(unsigned int end_stop);

    unsigned int get_position();

    unsigned int get_position_change();

    void reset_position();

    void set_position(unsigned int pos);

    MotorState get_state();

    MotorMode get_mode();

    void set_mode(MotorMode state);

    void disable();

    void cycle(unsigned long now) override;
};

#endif //ARDUINO_PROJECTS_ROOT_MOTOR_H
