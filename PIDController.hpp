#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

/// @brief PID制御を行う。フィードバック値を保持し、出力値を計算する。
class PIDController
{
    /// @brief PID制御のpゲイン
    const float kp;
    /// @brief PID制御のiゲイン
    const float ki;
    /// @brief PID制御のdゲイン
    const float kd;
    /// @brief 最大出力(積分器のanti-windup用)
    const float clamping_output;
    /// @brief update_output()の実行間隔
    const uint32_t interval;

    /// @brief モータの現在の角度(°)
    float angle;
    /// @brief モータの現在の回転数(rpm)
    int16_t rpm;
    /// @brief モータに現在実際に流れている電流量(mA)
    int16_t amp;
    /// @brief モータの現在の温度(℃)
    int8_t temp;

    /// @brief 制御目標値(rpm)
    int16_t target_rpm;

    float integral;
    float previous_error;
    uint32_t count;
public:
    /// @brief PIDControllerクラスのコンストラクタ
    /// @param kp pゲイン
    /// @param ki iゲイン
    /// @param kd dゲイン
    /// @param clamping_output 最大出力(積分器のanti-windup用)
    /// @param interval update_output()の実行間隔
    PIDController(float kp, float ki, float kd, float clamping_output, uint32_t interval);

    /// @brief フィードバック値を設定
    /// @param angle モータの現在の角度(°)
    /// @param rpm モータの現在の回転数(rpm)
    /// @param amp モータに現在実際に流れている電流量(mA)
    /// @param temp モータの現在の温度(℃)
    void set_feedback_values(float angle, int16_t rpm, int16_t amp, uint8_t temp);

    /// @brief 制御目標値を設定
    /// @param target_rpm 制御目標値(rpm)
    void set_target_rpm(int16_t target_rpm);

    /// @brief 内部状態からPID出力値を計算し、内部状態を更新
    /// @return 出力値
    float update_output();
};

#endif