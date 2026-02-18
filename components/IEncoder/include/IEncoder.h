// IEncoder.h
#pragma once
#include <stdint.h>

class IEncoder {
public:
    virtual ~IEncoder() = default;
    virtual void reset() = 0;
    virtual int32_t getAndClearDeltaTicks() = 0;   // ticks since last call
    virtual float ticksPerRev() const = 0;
};
