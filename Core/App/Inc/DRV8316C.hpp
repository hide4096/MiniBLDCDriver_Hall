#pragma once

#include "spi.h"
#include "tim.h"
#include "adc.h"

class DRV8316C
{
public:
    enum registerMap : uint8_t
    {
        IC_Status_Register = 0x00,
        Status_Register_1 = 0x01,
        Status_Register_2 = 0x02,
        Control_Register_1 = 0x03,
        Control_Register_2 = 0x04,
        Control_Register_3 = 0x05,
        Control_Register_4 = 0x06,
        Control_Register_5 = 0x07,
        Control_Register_6 = 0x08,
        Control_Register_10 = 0x09,
    };

    float currentGain = 1.2;

    DRV8316C(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc, GPIO_TypeDef *csPort, uint16_t csPin) : hspi(hspi), htim(htim), hadc(hadc), csPort(csPort), csPin(csPin) {}

    uint8_t init(float supplyVoltage = 6.0f)
    {
        // 入力電圧を設定
        _supplyVoltage = supplyVoltage;

        // CSピンをHIGHに設定
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

        uint8_t recv, send;

        // レジスタのロックを解除
        uint8_t REG_LOCK = 0b011;
        send = readRegister(Control_Register_1);
        send |= REG_LOCK;
        writeRegister(Control_Register_1, send);

        // ステータスを取得
        showStatus();

        // フォールトをクリア
        uint8_t CLR_FLT = 0b1;
        send = readRegister(Control_Register_2);
        send |= CLR_FLT;
        writeRegister(Control_Register_2, send);
        while (readRegister(Control_Register_2) & 1 != 0){
            HAL_Delay(1);
        }

        // 電流センサのゲインを変更
        uint8_t CSA_GAIN = 0b01; // 0.3V/A
        currentGain = 0.3;

        send = readRegister(Control_Register_5);
        send |= CSA_GAIN;
        writeRegister(Control_Register_5, send);

        if (readRegister(Control_Register_5) != send)
        {
            return 1;
        }

        uint8_t PWM_MODE = 0b00; // 3PWM mode
        send = readRegister(Control_Register_2);
        send |= PWM_MODE << 1;
        writeRegister(Control_Register_2, send);

        // ディレイを設定
        uint8_t DLY_TARGET = 0xF;
        uint8_t DLYCMP_EN = 0b0;
        send = readRegister(Control_Register_10);
        send |= DLYCMP_EN | DLY_TARGET << 4;
        writeRegister(Control_Register_10, send);

        // レジスタのロックを有効化
        REG_LOCK = 0b110;
        send = readRegister(Control_Register_1);
        send |= REG_LOCK;
        writeRegister(Control_Register_1, send);

        // PWMのデューティー比を0に設定
        _setDutyCycle(0, 0, 0);

        // ADCを起動
        HAL_ADC_Start_DMA(hadc, (uint32_t *)&batteryVoltageRaw, 1);

        // TIMの諸々を起動
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
        HAL_TIM_Base_Start(htim);

        // 電流センサのオフセットを取得
        const int offsetSamples = 100;
        for (int i = 0; i < offsetSamples; i++)
        {
            float _U, _V, _W;
            getCurrentRaw(&_U, &_V, &_W);
            currentOffsetRaw[0] += _U;
            currentOffsetRaw[1] += _V;
            currentOffsetRaw[2] += _W;
            HAL_Delay(1);
        }
        currentOffsetRaw[0] /= offsetSamples;
        currentOffsetRaw[1] /= offsetSamples;
        currentOffsetRaw[2] /= offsetSamples;

        return 0;
    }

    void showStatus()
    {
        printf("ICSTATUS:0x%02x\n", readRegister(IC_Status_Register));
        printf("STATUS1:0x%02x\n", readRegister(Status_Register_1));
        printf("STATUS2:0x%02x\n", readRegister(Status_Register_2));
    }

    void setVoltage(float U, float V, float W)
    {
        float _U = U / _supplyVoltage + 0.5f;
        float _V = V / _supplyVoltage + 0.5f;
        float _W = W / _supplyVoltage + 0.5f;

        _setDutyCycle(_U, _V, _W);
    }

    float getVoltage()
    {
        return (batteryVoltageRaw / 4096.0f) * 3.3f * 4.935f - 3.74f;
    }

    void getCurrentAmpere(float *U, float *V, float *W)
    {
        float _U, _V, _W;
        getCurrentRaw(&_U, &_V, &_W);

        *U = (_U - currentOffsetRaw[0]) * 3.3f / currentGain;
        *V = (_V - currentOffsetRaw[1]) * 3.3f / currentGain;
        *W = (_W - currentOffsetRaw[2]) * 3.3f / currentGain;
    }

private:
    SPI_HandleTypeDef *hspi;
    TIM_HandleTypeDef *htim;
    ADC_HandleTypeDef *hadc;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    float _supplyVoltage;
    uint16_t batteryVoltageRaw;

    float currentOffsetRaw[3];

    uint32_t period;

    void changeSPIPhase2Edge()
    {
        if (hspi->Init.CLKPhase != SPI_PHASE_2EDGE)
        {
            HAL_SPI_DeInit(hspi);
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
            HAL_SPI_Init(hspi);
        }
    }

    void getCurrentRaw(float *U, float *V, float *W)
    {
        *U = hadc->Instance->JDR1 / 4096.0f;
        *V = hadc->Instance->JDR2 / 4096.0f;
        *W = hadc->Instance->JDR3 / 4096.0f;
    }

    void _transferFrame(uint16_t sendFrame, uint16_t *recvFrame)
    {
        changeSPIPhase2Edge();
        HAL_Delay(1);

        HAL_StatusTypeDef status;
        uint16_t _sendFrame = sendFrame;
        uint16_t _recvFrame = 0;

        // Set Parity
        int parity = 0;
        uint16_t _tempFrame = sendFrame;
        while (_tempFrame > 0)
        {
            if (_tempFrame & 0b1)
            {
                parity = 1 - parity;
            }
            _tempFrame >>= 1;
        }
        _sendFrame |= parity << 8;

        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, (uint8_t *)&_sendFrame, (uint8_t *)&_recvFrame, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

        if (recvFrame != nullptr)
        {
            *recvFrame = _recvFrame;
        }
    }

    void _setDutyCycle(float U, float V, float W)
    {

        uint32_t _U = _constrainDuty(U);
        uint32_t _V = _constrainDuty(V);
        uint32_t _W = _constrainDuty(W);

        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, _U);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, _V);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, _W);
    }

    uint32_t _constrainDuty(float input)
    {
        uint32_t period = htim->Instance->ARR;
        int _input = input * period;
        if (_input < 0)
        {
            return 0;
        }
        else if (_input > period)
        {
            return period;
        }
        else
        {
            return _input;
        }
    }

    uint8_t readRegister(uint8_t reg)
    {
        uint16_t sendFrame = 0;
        uint16_t recvFrame = 0;

        sendFrame = 1 << 15;            // R/W bit
        sendFrame |= (reg & 0x3F) << 9; // Address
        sendFrame |= 0x0000;            // Data

        _transferFrame(sendFrame, &recvFrame);

        return recvFrame & 0xFF;
    }

    void writeRegister(uint8_t reg, uint8_t data)
    {
        uint16_t sendFrame = 0;

        sendFrame = 0 << 15;            // R/W bit
        sendFrame |= (reg & 0x3F) << 9; // Address
        sendFrame |= data;              // Data

        _transferFrame(sendFrame, nullptr);
    }
};