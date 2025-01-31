#include "app_main.h"
#include "DRV8316C.hpp"
#include "foc_control.hpp"

enum controlMode
{
    MANUAL,
    FOC,
    OFF
};

struct motorState
{
    char name[32];
    float calibrateVoltage;
    int polePairs;
    int zeroIndex;
    int isCW;
    bool needCalibration;
};

class HallSensor
{
public:
    int offset = 0;

    float readAngle()
    {
        int index = radIndex[readHallSensor() - 1] - offset;
        return index * M_PI / 3.0f;
    }

    int getIndex()
    {
        return radIndex[readHallSensor() - 1];
    }

private:
    const int radIndex[6] = {0, 2, 1, 4, 5, 3};

    uint8_t readHallSensor()
    {
        uint8_t hall = HAL_GPIO_ReadPin(U_GPIO_Port, U_Pin) | (HAL_GPIO_ReadPin(V_GPIO_Port, V_Pin) << 1) | (HAL_GPIO_ReadPin(W_GPIO_Port, W_Pin) << 2);
        return hall;
    }
};

const float deltaT = 1.f / 20000.f;
const float supplyVoltage = 16.0f;
volatile controlMode mode = OFF;

DRV8316C driver(&hspi1, &htim1, &hadc1, MOT_CS_GPIO_Port, MOT_CS_Pin);
HallSensor hall;

focControl foc(deltaT, supplyVoltage);
focControl::uvwVoltage calibration;
focControl::uvwVoltage output;

motorState motorInfo;

/*
    @brief モーター設定をフラッシュに書き込む
    @param _state モーター設定
    @return
        0:成功
        1:フラッシュ操作失敗
*/

int writeMotorState(const motorState *state)
{
    HAL_StatusTypeDef _status;
    uint32_t _address = 0x8060000;

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
    for (unsigned int i = 0; i < sizeof(motorState); i++)
    {
        _status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, _address + i, ((uint8_t *)state)[i]);
        if (_status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 1;
        }
    }

    HAL_FLASH_Lock();

    return 0;
}

/*
    @brief フラッシュからモーター設定を読み込む
    @return
        0:成功
        1:失敗
*/

int tryReadMotorState(motorState *state)
{
    uint32_t _address = 0x8060000;
    motorState _readState;

    memcpy(&_readState, (void *)_address, sizeof(motorState));
    if (_readState.polePairs > 0)
    {
        if (state != nullptr)
        {
            memcpy(state, &_readState, sizeof(motorState));
        }
    }
    else
    {
        return 1;
    }

    return 0;
}

volatile uint32_t loopClock = 0;
float angle = 0.0f;
float angleVel = 0.0f;
float integralGain = 0.0f;
float targetSpeed = M_PI_2;
float targetQ = 0.0f;

float kP = 10.f, kI = 0.0f, kD = 10.0f;

void app_main(void)
{

    printf("app_main()\n");

    // モーター設定読み込み
    if (tryReadMotorState(&motorInfo) != 0)
    {
        printf("Motor state is not set.\n");
        printf("Setting default values.\n");
        motorInfo = {
            "DCH-3514", // Name
            1.5f,       // Calibrate voltage
            7,          // Pole pairs
            0,          // Zero Index
            1,          // CW
            true,       // Need calibration
        };
    }
    printf("|Finish|\tMotor settings\n");

    // エンコーダーとモタドラ初期化
    if (driver.init(supplyVoltage) != 0)
    {
        printf("MotorDriver init failed.\n");
        return;
    }

    printf("|Finish|\tEncoder and MotorDriver init\n");

    // モーター設定を表示
    printf("\n----------------\n");
    printf("Motor settings\n");
    printf("----------------\n");
    printf("Name:%s\n", motorInfo.name);
    printf("Pole pairs:%d\n", motorInfo.polePairs);
    printf("CW:%d\n", motorInfo.isCW);
    printf("Zero angle:%d\n", motorInfo.zeroIndex);
    printf("Calibration voltage:%f\n", motorInfo.calibrateVoltage);
    printf("----------------\n");

    // 計測タイマー開始
    HAL_TIM_Base_Start(&htim5);
    htim5.Instance->CNT = 0;

    // 制御ループ開始
    mode = OFF;
    HAL_TIM_Base_Start_IT(&htim7);
    printf("|Finish|\tControl loop start\n");

    // モーター設定
    if (motorInfo.needCalibration)
    {
        printf("\n----------------\n");
        printf("Calibration start\n");
        printf("----------------\n");

        // キャリブレーション電圧を設定
        float calibVolt = motorInfo.calibrateVoltage;
        mode = MANUAL;

        /*
            電圧かけて電気角を0にする
            やりすぎるとアチアチになるので注意
        */
        float _ualpha, _ubeta;
        float theta = 0.0f;
        const float _3PI_2 = 3.f * M_PI / 2.f;
        const float SQRT3_2 = 0.86602540378f;
        theta = _3PI_2;

        _ualpha = -sinf(theta) * calibVolt;
        _ubeta = cosf(theta) * calibVolt;

        calibration.U = _ualpha;
        calibration.V = -0.5f * _ualpha + SQRT3_2 * _ubeta;
        calibration.W = -0.5f * _ualpha - SQRT3_2 * _ubeta;
        HAL_Delay(300);
        motorInfo.zeroIndex = hall.getIndex();
        float zeroAngle = hall.readAngle();
        printf("Zero Angle set\n");

        /*
            sin波で強制転流させて回転方向を確認
        */
        const int phaseLength = 1000;

        for (int i = 0; i <= phaseLength; i++)
        {
            // Uを最大にしたいので位相をずらす
            theta = _3PI_2 + M_PI * i / phaseLength;
            float _ualpha = -sinf(theta) * calibVolt;
            float _ubeta = cosf(theta) * calibVolt;
            calibration.U = _ualpha;
            calibration.V = -0.5f * _ualpha + SQRT3_2 * _ubeta;
            calibration.W = -0.5f * _ualpha - SQRT3_2 * _ubeta;
            HAL_Delay(1);
        }
        // 回した後にエンコーダの角度を見て回転方向を確認
        if (hall.readAngle() - zeroAngle < M_PI)
        {
            motorInfo.isCW = 1;
        }
        else
        {
            motorInfo.isCW = -1;
        }

        mode = OFF;

        printf("Calibration done\n");
        printf("CW:%d\n", motorInfo.isCW);
        printf("----------------\n");
        motorInfo.needCalibration = false;
        if (writeMotorState(&motorInfo) != 0)
        {
            printf("Write motor state failed\n");
        }
    }
    hall.offset = motorInfo.zeroIndex;

    mode = OFF;

    HAL_Delay(1000);
    printf("Start FOC\n");
    foc.reset();
    foc.setGain(0.2f, 3.0f, 0.f, 0.2f, 3.0f, 0.0f);
    foc.targetCurrentDQ.D = 0.0f;
    foc.targetCurrentDQ.Q = 3.0f;

    mode = FOC;
    // mode = OFF;

    while (1)
    {
        printf(">angleVel:%.2f\n",angleVel);
    }
}

/*
    ADC1
        TIM1のカウントリセット時にinjectionModeでCH10, CH11, CH12を読み込む
    TIM1
        モーター用PWM
        Center-aligned mode 1
        カウントリセット時にイベントを発生
    TIM7
        モーター制御ループ
*/

inline float updateLPF(float input, float prev, float Tf)
{
    float alpha = Tf / (Tf + deltaT);
    return alpha * prev + (1.0f - alpha) * input;
}

volatile float prevMeasuredAngle = 0.0f;
volatile float prevAngleVel = 0.0f;
volatile float integralAngleVel = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // モーター制御ループ
    if (htim->Instance == TIM7)
    {
        // 計測タイマーリセット
        htim5.Instance->CNT = 0;

        // 角度と角速度を計算
        float _measuredAngle = hall.readAngle() * motorInfo.isCW;

        float deltaAngle = _measuredAngle - prevMeasuredAngle;
        if (deltaAngle > M_PI)
        {
            deltaAngle -= 2.f * M_PI;
        }
        else if (deltaAngle < -M_PI)
        {
            deltaAngle += 2.f * M_PI;
        }
        deltaAngle /= motorInfo.polePairs;
        angle += deltaAngle;
        angleVel = updateLPF(deltaAngle / deltaT, angleVel, 0.3f);
        prevMeasuredAngle = _measuredAngle;

        float angleVelError = targetSpeed - angleVel;

        integralAngleVel += angleVelError * deltaT;

        const float maxIntegralAngleVel = 10.0f;
        if (integralAngleVel > maxIntegralAngleVel)
        {
            integralAngleVel = maxIntegralAngleVel;
        }
        else if (integralAngleVel < -maxIntegralAngleVel)
        {
            integralAngleVel = -integralAngleVel;
        }

        targetQ = angleVelError * kP + integralAngleVel * kI + (prevAngleVel - angleVel) * kD;
        prevAngleVel = angleVel;

        foc.targetCurrentDQ.Q = -targetQ;

        foc.electricalAngle = _measuredAngle;
        driver.getCurrentAmpere(&foc.current.U, &foc.current.V, &foc.current.W);

        if (mode == FOC)
        {
            foc.loop(&output.U, &output.V, &output.W);
        }
        else
        {
            foc.reset();
            switch (mode)
            {
            case MANUAL:
                output.U = calibration.U;
                output.V = calibration.V;
                output.W = calibration.W;
                break;
            case OFF:
            default:
                output.U = 0.0f;
                output.V = 0.0f;
                output.W = 0.0f;
                break;
            }
        }
        driver.setVoltage(output.U, output.V, output.W);
        loopClock = htim5.Instance->CNT;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == nFAULT_Pin)
    {
        HAL_TIM_Base_Stop_IT(&htim7);
        mode = OFF;
    }
}
