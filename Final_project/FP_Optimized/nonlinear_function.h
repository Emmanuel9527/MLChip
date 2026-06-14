#ifndef NONLINEAR_FUNCTION_H
#define NONLINEAR_FUNCTION_H

#include "systemc.h"

// PE nonlinear-function unit behavior model.
// ReLU is used after convolution and FC. Max selection is used by pooling.
SC_MODULE(NonlinearFunction)
{
    unsigned long long relu_ops;
    unsigned long long max_ops;

    float relu(float value, bool enable)
    {
        if (!enable)
            return value;
        relu_ops++;
        return value < 0.0f ? 0.0f : value;
    }

    float max_select(float a, float b)
    {
        max_ops++;
        return a > b ? a : b;
    }

    SC_CTOR(NonlinearFunction)
    {
        relu_ops = 0;
        max_ops = 0;
    }
};

#endif
