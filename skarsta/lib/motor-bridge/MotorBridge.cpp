#include "MotorBridge.h"

void MotorBridge::begin() {
    Motor::begin();
    initPin(r_enable);
    initPin(l_enable);
    initPin(r_pwm);
    initPin(l_pwm);
    this->enable();
}

void MotorBridge::_off() {
    digitalWrite(r_enable, LOW);
    digitalWrite(l_enable, LOW);
    this->speed = 0;
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, 0);
}

void MotorBridge::enable() {
    digitalWrite(r_enable, HIGH);
    digitalWrite(l_enable, HIGH);
}

void MotorBridge::_dir_cw() {
    setSpeed(CW, MIN_SPEED);
    this->enable();
}

void MotorBridge::_dir_ccw() {
    setSpeed(CCW, MIN_SPEED);
    this->enable();
}

void MotorBridge::setSpeed(MotorState state, uint8_t speed) {
    if (state == OFF) {
        return;
    }
    this->speed = speed;

#ifdef __DEBUG__
    Serial.print("m speed ");
    Serial.print(speed);
    Serial.println();
#endif

    if (state == CCW) {
        analogWrite(l_pwm, 0);
        analogWrite(r_pwm, speed);
    } else if (state == CW) {
        analogWrite(r_pwm, 0);
        analogWrite(l_pwm, speed);
    }
}

void MotorBridge::cycle(unsigned long now) {
    static unsigned long last_tick = now;
    if (get_period(last_tick, now) >= SPEED_STEP_DURATION && speed < MAX_SPEED && speed >= MIN_SPEED) {
        setSpeed(get_state(), speed + 5 % (MAX_SPEED + 1));
        last_tick = now;
    }

    Motor::cycle(now);
}