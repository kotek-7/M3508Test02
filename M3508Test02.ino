#include <mcp_can.h>
#include <SPI.h>
#include <PIDController.hpp>

// [SPIメモ]
// CS: pin 10
// MISO: pin 12
// MOSI: pin 11
// SCK: pin 13
MCP_CAN CAN(10); // CAN CS: pin 10

constexpr uint16_t can_id = 0x200;
constexpr uint32_t can_send_interval = 100;
constexpr uint32_t can_receive_interval = 100;
constexpr uint32_t serial_read_interval = 100;

// PID制御用定数
constexpr float kp = 0.5f;
constexpr float ki = 0.0003f;
constexpr float kd = 40;
constexpr float clamping_output = 1000;

/// @brief 前回のCAN送信時間
uint32_t previous_can_send_millis;
/// @brief 前回のCAN受信時間
uint32_t previous_can_receive_millis;
/// @brief 前回のシリアル受信時間
uint32_t previous_serial_read_millis;

void setup()
{
    pinMode(13, OUTPUT);
    Serial.begin(115200);

    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
        Serial.println("Init OK!");
        CAN.setMode(MCP_NORMAL);
    }
    else
    {
        Serial.println("Init Fail!");
        Serial.println("");
    }

    previous_can_send_millis = millis();
    previous_can_receive_millis = millis();
}

void loop()
{
    static PIDController pid_controller{kp, ki, kd, clamping_output, can_send_interval};

    /// @brief モータに送信する電流値(mA)
    static int32_t command_currents[4] = {500, 0, 0, 0};

    /// @brief loop()の実行回数
    static uint32_t count;

    // CANで制御量を送信
    if ((millis() - previous_can_send_millis) > can_send_interval)
    {
        command_currents[0] = pid_controller.update_output();
        uint8_t tx_buf[8];
        milli_amperes_to_bytes(command_currents, tx_buf);

        CAN.sendMsgBuf(can_id, 0, 8, tx_buf);

        previous_can_send_millis = millis();
    }

    // CANでフィードバック値を受信
    if ((millis() - previous_can_receive_millis) > can_receive_interval)
    {
        if (CAN.checkReceive() == CAN_MSGAVAIL)
        {
            uint32_t rx_id;
            uint8_t length;
            uint8_t rx_buf[8];
            CAN.readMsgBuf(reinterpret_cast<unsigned long *>(&rx_id), &length, rx_buf); // 明示的に型キャストしないとなぜかLSPに怒られる

            uint32_t controller_id = rx_id - 0x200;
            float angle;
            int16_t rpm;
            int16_t amp;
            uint8_t temp;
            derive_feedback_fields(rx_buf, &angle, &rpm, &amp, &temp);

            pid_controller.set_feedback_values(angle, rpm, amp, temp);
        }
        previous_can_receive_millis = millis();
    }

    // シリアル通信で制御目標値を受信
    if ((millis() - previous_serial_read_millis) > serial_read_interval)
    {
        if (Serial.available())
        {
            delay(1); // 一連のシリアル信号をすべて受信するまで待つ
            String input_string = "";
            while (Serial.available() > 0)
            {
                char input_char = Serial.read();
                input_string.concat(input_char);
            }
            pid_controller.set_target_rpm(input_string.toInt());
            Serial.print("Set target rpm to: ");
            Serial.print(input_string);
            Serial.print("\n\n");
        }
        previous_serial_read_millis = millis();
    }

    digitalWrite(13, HIGH);
    count++;
}

/// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
/// @param milli_amperes 4つの-20000\~20000の電流値(mA)を格納した配列 (要素番号と速度コントローラIDが対応)
/// @param out_tx_buf 結果の書き込み用配列
void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
        uint8_t upper = (milli_ampere >> 8) & 0xFF;
        uint8_t lower = milli_ampere & 0xFF;
        out_tx_buf[i * 2] = upper;
        out_tx_buf[i * 2 + 1] = lower;
    }
}

/// @brief 速度コントローラから受け取ったデータから、フィードバック値を導出
/// @param rx_buf CANで受信した配列
/// @param out_angle ロータの角度(0°\~360°) (結果書き込み用)
/// @param out_rpm 回転速度(rpm) (結果書き込み)
/// @param out_amp 実際のトルク電流(?) (結果書き込み用)
/// @param out_temp モータの温度(℃) (結果書き込み用)
void derive_feedback_fields(const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp)
{
    *out_angle = (float)(rx_buf[0] << 8 | rx_buf[1]) * 360.0f / 8191.0f;
    *out_rpm = rx_buf[2] << 8 | rx_buf[3];
    *out_amp = rx_buf[4] << 8 | rx_buf[5];
    *out_temp = rx_buf[6];
}
