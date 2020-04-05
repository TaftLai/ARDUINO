#include <Motor.h>

#ifdef __EEPROM__
#include <EEPROM.h>
#endif

static Motor *motor = nullptr;

static unsigned int position_abs(unsigned int a, unsigned int b) {
    return a >= b ? a - b : b - a;
}

Motor::Motor(uint8_t _pin1, uint8_t _pin2) :
        sensor((char) _pin1, (char) _pin2), sensor_pin_1(_pin1), sensor_pin_2(_pin2) {
    motor = this;
}

void Motor::begin() {
#ifdef __EEPROM__
#ifndef __ULTRASONIC__
    EEPROM.get(ADDRESS_POSITION, position);
#endif
    EEPROM.get(ADDRESS_END_STOP_0, end_stop[0]);
    EEPROM.get(ADDRESS_END_STOP_1, end_stop[1]);
    EEPROM.get(ADDRESS_MODE, mode);
#endif

#ifdef __DEBUG__
#ifndef __ULTRASONIC__
    Serial.print("m pos: ");
    Serial.println(position);
#endif
    Serial.print("m end_pos: ");
    Serial.print(end_stop[0]);
    Serial.print(", ");
    Serial.println(end_stop[1]);
    Serial.print("m mode:");
    Serial.println(mode);
#endif

#ifndef __ULTRASONIC__
    auto interrupt_routine = []() {
        motor->update_position(motor->sensor.process());
    };
    attachInterrupt((uint8_t) digitalPinToInterrupt(sensor_pin_1), interrupt_routine, CHANGE); // set interrupt
    attachInterrupt((uint8_t) digitalPinToInterrupt(sensor_pin_2), interrupt_routine, CHANGE); // set interrupt
#endif
}

void Motor::off() {
    _off();
#ifdef __DEBUG__
    Serial.print("m off ");
    Serial.print(end_stop[0]);
    Serial.print(", ");
    Serial.print(end_stop[1]);
    Serial.println();
#endif
    state = OFF;
}

void Motor::dir_cw() {
    if (disabled || (get_position() >= end_stop[1] && mode == CALIBRATED)) {
        return;
    }

#ifdef __DEBUG__
    Serial.println("m cw");
#endif
    _dir_cw();
    state = CW;
}

void Motor::dir_ccw() {
    if (disabled || (get_position() <= end_stop[0] && mode != UNCALIBRATED)) {
        return;
    }

#ifdef __DEBUG__
    Serial.println("m ccw");
#endif
    _dir_ccw();
    state = CCW;
}

unsigned int Motor::get_position() {
#ifndef __ULTRASONIC__
    return position;
#else
    return (unsigned int) sensor.Ranging(CM);
#endif
}

void Motor::reset_position() {
    if (disabled) return;
#ifndef __ULTRASONIC__
    this->position = 0u;
#else
    this->end_stop[0] = get_position();
#endif

#ifdef __EEPROM__
#ifndef __ULTRASONIC__
    updateEEPROM(ADDRESS_POSITION, position);
#endif
    updateEEPROM(ADDRESS_END_STOP_0, this->end_stop[0]);
#endif
#ifdef __DEBUG__
    Serial.println("m rst");
#endif
}

void Motor::set_position(unsigned int pos) {
    if (mode != CALIBRATED || position_abs(pos, get_position()) < MINIMUM_POS_CHANGE) {
        return;
    }
    next_position = pos;
    if (next_position < get_position()) {
        this->dir_ccw();
    } else {
        this->dir_cw();
    }
#ifdef __DEBUG__
    Serial.print("m pos:");
    Serial.println(pos);
#endif
}

#ifndef __ULTRASONIC__

void Motor::update_position(const unsigned char result) {
    position_change++;
    if (mode == UNCALIBRATED) {
        return;
    }

    if (result == DIR_CW) {
        position++;
    } else if (result == DIR_CCW && position != 0) {
        position--;
    }
}

#endif

MotorState Motor::get_state() {
    return this->state;
}

MotorMode Motor::get_mode() {
    return this->mode;
}

void Motor::set_mode(MotorMode mode) {
    if (disabled) return;
    this->mode = mode;
#ifdef __EEPROM__
    updateEEPROM(ADDRESS_MODE, this->mode);
#endif
#ifdef __DEBUG__
    Serial.print("m mode: ");
    Serial.println(mode);
#endif
}

void Motor::set_end_stop(unsigned int end_stop) {
    if (disabled) return;
    this->end_stop[1] = end_stop;
#ifdef __EEPROM__
    updateEEPROM(ADDRESS_END_STOP_1, this->end_stop[1]);
#endif
#ifdef __DEBUG__
    Serial.print("m end_pos: ");
    Serial.println(end_stop);
#endif
}

void Motor::initPin(uint8_t pin, uint8_t val) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, val);
}

void Motor::cycle(unsigned long now) {
    if (mode == UNCALIBRATED) {
        return;
    }

#if defined(__EEPROM__) && !defined(__ULTRASONIC__)
    updateEEPROM(ADDRESS_POSITION, position);
#endif

    if ((get_state() == CCW && get_position() <= end_stop[0] + STOP_POS_DIFF) ||
        (get_state() == CW && get_position() >= end_stop[1] - STOP_POS_DIFF) ||
        (next_position >= 0 && position_abs(get_position(), (unsigned int) next_position) <= STOP_POS_DIFF)) {
        off();
        next_position = -1;
    }
};

void Motor::disable() {
    disabled = true;
    off();
}

unsigned int Motor::get_position_change() {
#ifndef __ULTRASONIC__
    unsigned int cur_change = position_change;
    position_change = 0;
    return cur_change;
#else
    const unsigned int cur_pos = get_position(), cur_change = position_change;
    position_change = 0;
    return cur_pos > cur_change ? cur_pos - cur_change : cur_change - cur_pos;
#endif
}
