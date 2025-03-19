#pragma once

#define FLASH_START_ADDRESS 0x8060000

inline float constrain(float value, float absoulteLimit)
{
    if (value > absoulteLimit)
    {
        return absoulteLimit;
    }
    else if (value < -absoulteLimit)
    {
        return -absoulteLimit;
    }
    return value;
}

class PID
{
public:
    PID() {}

    struct PIDgain
    {
        float kP;
        float kI;
        float kD;
        float integralLimit;
    };

    PIDgain gain;

    void init(float deltaT)
    {
        this->deltaT = deltaT;
    }

    void setGain(PIDgain* gain)
    {
        this->gain = *gain;
        reset();
    }

    void reset()
    {
        integral = 0.0f;
        prevError = 0.0f;
    }

    float update(float error)
    {
        integral += error * deltaT;
        integral = constrain(integral, gain.integralLimit);

        float dError = (error - prevError) / deltaT;
        prevError = error;
        return gain.kP * error + gain.kI * integral + gain.kD * dError;
    }

private:
    float integral = 0.0f, prevError = 0.0f, deltaT = 1.0f;
};

class LPF
{
public:
    LPF() {}

    void init(float Tf, float deltaT)
    {
        alpha = Tf / (Tf + deltaT);
    }

    float update(float input, float prev)
    {
        float output = alpha * prev + (1.0f - alpha) * input;
        return output;
    }

private:
    float alpha = 0.0f;
};

int writeFlash(const void *data, uint32_t size)
{
    HAL_StatusTypeDef _status;
    uint32_t _address = FLASH_START_ADDRESS;

    HAL_FLASH_Unlock();

    // フラッシュ消去
    uint32_t _error = 0;
    FLASH_EraseInitTypeDef _eraseInit;
    _eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    _eraseInit.Sector = FLASH_SECTOR_7;
    _eraseInit.NbSectors = 1;
    _eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&_eraseInit, &_error);
    if (_error != 0xFFFFFFFF)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    // 1バイトずつ書き込み
    for (uint32_t i = 0; i < size; i++)
    {
        _status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, _address + i, ((uint8_t *)data)[i]);
        if (_status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 1;
        }
    }

    HAL_FLASH_Lock();

    return 0;
}

void readFlash(void *data, uint32_t size)
{
    uint32_t _address = FLASH_START_ADDRESS;
    memcpy(data, (void *)_address, size);
}