#pragma once

class focControl
{
public:
    float batteryVoltage;
    float electricalAngle;

    struct uvwVoltage
    {
        float U;
        float V;
        float W;
    };

    struct uvmAmpere
    {
        float U;
        float V;
        float W;
    };

    struct PID
    {
        float kp;
        float ki;
        float kd;

        float integral;
        float prevError;
    };

    struct dqVoltage
    {
        float d;
        float q;
    };

    struct dqAmpere
    {
        float D;
        float Q;
    };

    PID pidD, pidQ;

    uvmAmpere current;
    dqAmpere targetCurrentDQ;
    dqAmpere measuredCurrentDQ;
    dqAmpere errorCurrentDQ;

    dqVoltage output;

    focControl(float deltaT, float supplyVoltage) : deltaT(deltaT), supplyVoltage(supplyVoltage) {}

    void reset()
    {
        pidD.integral = pidQ.integral = 0.f;
        pidD.prevError = pidQ.prevError = 0.f;
    }

    void setGain(float kp_d, float ki_d, float kd_d, float kp_q, float ki_q, float kd_q)
    {
        pidD.kp = kp_d;
        pidD.ki = ki_d;
        pidD.kd = kd_d;

        pidQ.kp = kp_q;
        pidQ.ki = ki_q;
        pidQ.kd = kd_q;

        reset();
    }

    inline float updateLPF(float input, float prev)
    {
        float alpha = Tf / (Tf + deltaT);
        return alpha * prev + (1.0f - alpha) * input;
    }

    inline float constrain(float value, float min, float max)
    {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    void loop(float *u, float *v, float *w)
    {
        float sinE = sinf(electricalAngle);
        float cosE = cosf(electricalAngle);

        // 逆クラーク変換
        float _mid = (current.U + current.V + current.W) / 3.f;
        float _currentA = current.U - _mid;
        float _currentB = current.V - _mid;

        float currAlpha = _currentA;
        float currBeta = _currentA * _1_SQRT3 + _currentB * _2_SQRT3;

        // パーク変換
        dqAmpere _rawCurrentDQ;
        _rawCurrentDQ.D = currAlpha * cosE + currBeta * sinE;
        _rawCurrentDQ.Q = -currAlpha * sinE + currBeta * cosE;
        measuredCurrentDQ.D = updateLPF(_rawCurrentDQ.D, prevCurrentDQ.D);
        measuredCurrentDQ.Q = updateLPF(_rawCurrentDQ.Q, prevCurrentDQ.Q);

        prevCurrentDQ = measuredCurrentDQ;

        errorCurrentDQ.D = measuredCurrentDQ.D - targetCurrentDQ.D;
        errorCurrentDQ.Q = measuredCurrentDQ.Q - targetCurrentDQ.Q;

        pidD.integral += errorCurrentDQ.D * deltaT;
        pidQ.integral += errorCurrentDQ.Q * deltaT;

        float _limit = 10.0f;
        pidD.integral = constrain(pidD.integral, -_limit, _limit);
        pidQ.integral = constrain(pidQ.integral, -_limit, _limit);

        output.d = pidD.kp * errorCurrentDQ.D + pidD.ki * pidD.integral + pidD.kd * (errorCurrentDQ.D - pidD.prevError) / deltaT;
        output.q = pidQ.kp * errorCurrentDQ.Q + pidQ.ki * pidQ.integral + pidQ.kd * (errorCurrentDQ.Q - pidQ.prevError) / deltaT;

        pidD.prevError = errorCurrentDQ.D;
        pidQ.prevError = errorCurrentDQ.Q;

        // 逆パーク変換
        float alpha = output.d * cosE - output.q * sinE;
        float beta = output.d * sinE + output.q * cosE;

        // クラーク変換
        *u = constrain(alpha, -_limit, _limit);
        *v = constrain(-0.5f * alpha + SQRT3_2 * beta, -_limit, _limit);
        *w = constrain(-0.5f * alpha - SQRT3_2 * beta, -_limit, _limit);
    }

    float min3(float a, float b, float c)
    {
        return _min(a, _min(b, c));
    }

private:
    const float SQRT3_2 = 0.86602540378f;
    const float SQRT2_3 = 0.81649658092f;
    const float _2_SQRT3 = 1.15470053838f;
    const float _1_SQRT3 = 0.57735026919f;

    float deltaT = 1.f;
    float supplyVoltage = 1.0f;
    float Tf = 0.001f;
    dqAmpere prevCurrentDQ = {0.f, 0.f};

    float _min(float a, float b)
    {
        return a < b ? a : b;
    }

    float _max(float a, float b)
    {
        return a > b ? a : b;
    }

    float prevErrorD = 0.f, prevErrorQ = 0.f;
};