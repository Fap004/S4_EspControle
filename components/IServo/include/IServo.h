// IServo.h  (direction)
#pragma once

class IServo {
public:
    virtual ~IServo() = default;
    virtual void setAngleDeg(float angle) = 0;     // e.g. -30..+30
};
