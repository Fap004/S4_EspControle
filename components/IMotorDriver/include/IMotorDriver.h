// IMotorDriver.h
#pragma once
#include <stdint.h>
enum class MotorDir { Forward, Reverse, Brake, Coast };

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    virtual void setDuty(float duty01) = 0;        // 0.0 .. 1.0
    virtual void setDirection(MotorDir dir) = 0;
};
