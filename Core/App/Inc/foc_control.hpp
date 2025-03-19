#pragma once

#include "utils.hpp"

class focControl
{
public:
    struct uvw
    {
        float U;
        float V;
        float W;
    };

    struct dq
    {
        float D;
        float Q;
    };

    struct FOCInput
    {
        float electricalAngle;
        uvw current;
        dq targetCurrent;
    };

    PID pidCurrentD;
    PID pidCurrentQ;
    LPF lpfCurrentD;
    LPF lpfCurrentQ;

    dq rawCurrent, filterdCurrent;

    focControl() {}

    void init(float deltaT, float Tf, float supplyVoltage, int isCW)
    {
        this->supplyVoltage = supplyVoltage;
        this->isCW = isCW;
        pidCurrentD.init(deltaT);
        pidCurrentQ.init(deltaT);
        lpfCurrentD.init(Tf, deltaT);
        lpfCurrentQ.init(Tf, deltaT);
    }

    void reset()
    {
        filterdCurrent.D = 0.0f;
        filterdCurrent.Q = 0.0f;
        pidCurrentD.reset();
        pidCurrentQ.reset();
    }

    uvw loop(FOCInput *input)
    {
        float sinAngle = sinf(input->electricalAngle);
        float cosAngle = cosf(input->electricalAngle);

        // 逆クラーク変換
        float currentAlpha = (2.0f / 3.0f) * (input->current.U - 0.5f * input->current.V - 0.5f * input->current.W);
        float currentBeta = (2.0f / 3.0f) * (SQRT3_2 * (input->current.V - input->current.W));

        // パーク変換
        rawCurrent.D = currentAlpha * cosAngle + currentBeta * sinAngle;
        rawCurrent.Q = -currentAlpha * sinAngle + currentBeta * cosAngle;

        filterdCurrent.D = lpfCurrentD.update(rawCurrent.D, filterdCurrent.D);
        filterdCurrent.Q = lpfCurrentQ.update(rawCurrent.Q, filterdCurrent.Q);

        dq outputCurrent = {
            pidCurrentD.update(input->targetCurrent.D * isCW - filterdCurrent.D),
            pidCurrentQ.update(input->targetCurrent.Q * isCW - filterdCurrent.Q)};

        // 逆パーク変換
        float alpha = outputCurrent.D * cosAngle - outputCurrent.Q * sinAngle;
        float beta = outputCurrent.D * sinAngle + outputCurrent.Q * cosAngle;

        // クラーク変換
        float _voltageLimit = supplyVoltage / 8.0f;
        uvw outputVoltage{
            constrain(alpha, _voltageLimit),
            constrain(-0.5f * alpha + SQRT3_2 * beta, _voltageLimit),
            constrain(-0.5f * alpha - SQRT3_2 * beta, _voltageLimit)};

        return outputVoltage;
    }

private:
    const float SQRT3_2 = 0.86602540378f;
    const float SQRT2_3 = 0.81649658092f;
    const float _2_SQRT3 = 1.15470053838f;
    const float _1_SQRT3 = 0.57735026919f;

    float supplyVoltage = 1.0f;
    int isCW = 1;
};