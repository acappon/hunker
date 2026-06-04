#pragma once
// Accessors for the lgpio / hardware stubs used in tests
namespace stubs
{
    int  lastGpio();
    int  lastValue();
    void setWriteFail(bool fail);
    void reset();
} // namespace stubs
