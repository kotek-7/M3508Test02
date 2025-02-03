#include <mcp_can.h>
#include <SPI.h>

constexpr uint16_t can_id = 0x200;
constexpr uint32_t can_send_interval = 100;
constexpr uint32_t can_receive_interval = 100;
constexpr uint32_t serial_read_interval = 100;

MCP_CAN CAN(10); // CAN CS: pin 10

// [SPIメモ]
// CS: pin 10
// MISO: pin 12
// MOSI: pin 11
// SCK: pin 13

int32_t sending_milli_amperes[4] = {500, 0, 0, 0};

void scale_current_to_range(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);
void derive_feedback_fields(const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp);

/// @brief 前回のCAN送信時間
uint32_t previous_send_millis;
/// @brief 前回のCAN受信時間
uint32_t previous_receive_millis;

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

    previous_send_millis = millis();
    previous_receive_millis = millis();
}

void loop()
{
    /// @brief ループ回数
    static uint32_t count;

    // フィードバック値
    /// @brief モータの現在の角度(°)
    static float angle;
    /// @brief モータの現在の回転数(rpm)
    static int16_t rpm;
    /// @brief モータに現在実際に流れている電流量(mA)
    static int16_t amp;
    /// @brief モータの現在の温度(℃)
    static uint8_t temp;

    /// @brief 制御目標値(rpm)
    static int16_t target_rpm = 1000;

    // PID制御用フィールド
    static const float kp = 0.5f;
    static const float ki = 0.0003f;
    static const float kd = 40;
    static const float clamping_output = 1000;
    static float integral;
    static float previous_error;
    static float previous_integral;

    // CANで制御量を送信
    if ((millis() - previous_send_millis) > can_send_interval)
    {
        float current_error = static_cast<float>(target_rpm - rpm);
        integral += current_error * static_cast<float>(can_send_interval);
        float derivative = (current_error - previous_error) / static_cast<float>(can_send_interval);
        float raw_output = kp * current_error + ki * integral + kd * derivative;

        float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
        if (raw_output != clamped_output && (raw_output * current_error > 0))
        {
            integral = 0;
        }

        sending_milli_amperes[0] = clamped_output;

        if (count % 20 == 0)
        {
            Serial.print("target: ");
            Serial.print(target_rpm);
            Serial.print("rpm, ");
            Serial.print("error: ");
            Serial.print(current_error);
            Serial.print("rpm, ");
            Serial.print("output: ");
            Serial.print(clamped_output);
            Serial.print("mA, ");
            Serial.print("p: ");
            Serial.print(kp * current_error);
            Serial.print(", ");
            Serial.print("i: ");
            Serial.print(ki * integral);
            Serial.print(", ");
            Serial.print("di: ");
            Serial.print(ki * current_error * can_send_interval);
            Serial.print(", ");
            Serial.print("d: ");
            Serial.print(kd * derivative);
            Serial.print("\n");
        }

        previous_error = current_error;
        previous_integral = integral;

        uint8_t tx_buf[8];
        scale_current_to_range(sending_milli_amperes, tx_buf);
        CAN.sendMsgBuf(can_id, 0, 8, tx_buf);
        // if (count % 20 == 0)
        // {
        //     Serial.print("Send ID: 0x200");
        //     Serial.print("\n");
        //     Serial.print("tx_buf: ");
        //     for (uint8_t i = 0; i < 8; i++)
        //     {
        //         Serial.print(tx_buf[i], HEX);
        //         Serial.print(" ");
        //     }
        //     Serial.print("\n");
        // }
        previous_send_millis = millis();
        if (count % 20 == 0)
        {
            Serial.print("\n");
        }
    }

    // CANでフィードバック値を受信
    if ((millis() - previous_receive_millis) > can_receive_interval)
    {
        if (CAN.checkReceive() == CAN_MSGAVAIL)
        {
            uint32_t rx_id;
            uint8_t length;
            uint8_t rx_buf[8];
            CAN.readMsgBuf(reinterpret_cast<unsigned long*>(&rx_id), &length, rx_buf);
            // if (count % 20 == 0)
            // {
            //     Serial.print("Receive ID: ");
            //     Serial.print(rx_id, HEX);
            //     Serial.print("\n");
            //     Serial.print("Data: ");
            //     for (uint8_t i = 0; i < length; i++)
            //     {
            //         Serial.print(rx_buf[i], HEX);
            //         Serial.print(" ");
            //     }
            //     Serial.print("\n");
            // }

            uint32_t controller_id = rx_id - 0x200;

            derive_feedback_fields(rx_buf, &angle, &rpm, &amp, &temp);

            if (count % 20 == 0)
            {
                Serial.print("angle: ");
                Serial.print(angle);
                Serial.print(", ");
                Serial.print("rpm: ");
                Serial.print(rpm);
                Serial.print(", ");
                Serial.print("amp: ");
                Serial.print(amp);
                Serial.print(", ");
                Serial.print("temp: ");
                Serial.print(temp);
                Serial.print("\n");
            }
        }
        previous_receive_millis = millis();
        if (count % 20 == 0)
        {
            Serial.print("\n");
        }
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
            int32_t input = input_string.toInt();
            target_rpm = input;
            Serial.print("Make target rpm to: ");
            Serial.print(target_rpm);
            Serial.print("\n");
            Serial.print("\n");
        }
    }

    digitalWrite(13, HIGH);
    count++;
}

/// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
/// @param milli_amperes 4つの-20000\~20000の電流値(mA)を格納した配列 (要素番号と速度コントローラIDが対応)
/// @param out_tx_buf 結果の書き込み用配列
void scale_current_to_range(const int32_t milli_amperes[4], uint8_t out_tx_buf[8])
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
