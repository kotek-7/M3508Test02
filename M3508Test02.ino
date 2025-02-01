#include <mcp_can.h>
#include <SPI.h>

#define CAN_ID (0x200)
#define CAN_SEND_INTERVAL (100)
#define CAN_RECEIVE_INTERVAL (1000)

MCP_CAN CAN(10); // CAN CS: pin 10
// CS: pin 10
// MISO: pin 12
// MOSI: pin 11
// SCK: pin 13

const signed long sendingMilliAmperes[4] = {500, 0, 0, 0};

void scaleCurrentToRange(const signed long milliAmperes[4], byte outTxBuf[8]);
void deriveFeedbackFields(const byte rxBuf[8], float *outAngle, unsigned int *outRpm, signed int *outAmp, unsigned int *outTemp);

/// @brief 前回のアップデート時間
long previousSendMillis;
long previousReceiveMillis;

void setup()
{
    pinMode(13, OUTPUT);
    Serial.begin(115200);

    // init CAN bus, baudrate: 1000kbps@8MHz
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
        Serial.println("Init OK!");
        CAN.setMode(MCP_NORMAL);
    }
    else
    {
        Serial.println("Init Fail!");
    }

    previousSendMillis = millis();
    previousReceiveMillis = millis();
}

void loop()
{
    static float angle;
    static unsigned int rpm;
    static signed int amp;
    static unsigned int temp;

    static int targetRpm = 100;

    static const float kp = 0.001f;
    static const float ki = 0.0f;
    static const float kd = 0.0f;
    static float integral;
    static float previousError;

    if ((millis() - previousSendMillis) > CAN_SEND_INTERVAL)
    {
        signed int currentError = targetRpm - rpm;
        integral += currentError * CAN_SEND_INTERVAL;
        float derivative = (currentError - previousError) / CAN_SEND_INTERVAL;
        float output = kp * currentError + ki * integral + kd * derivative;
        previousError = currentError;
        output = min(output, 5000); // 安全装置

        // Serial.print("output: ");
        // Serial.print(output);
        // Serial.print("\n");

        byte txBuf[8];
        scaleCurrentToRange(sendingMilliAmperes, txBuf);
        CAN.sendMsgBuf(CAN_ID, 0, 8, txBuf);
        // Serial.print("Send ID: 0x200");
        // Serial.print("\n");
        // Serial.print("txBuf: ");
        // for (byte i = 0; i < 8; i++)
        // {
        //     Serial.print(txBuf[i], HEX);
        //     Serial.print(" ");
        // }
        // Seril.print("\n");
        previousSendMillis = millis();
    }

    if ((millis() - previousReceiveMillis) > CAN_RECEIVE_INTERVAL)
    {
        if (CAN.checkReceive() == CAN_MSGAVAIL)
        {
            unsigned long rxId;
            byte length;
            byte rxBuf[8];
            CAN.readMsgBuf(&rxId, &length, rxBuf);
            Serial.print("Receive ID: ");
            Serial.print(rxId, HEX);
            Serial.print("\n");
            Serial.print("Data: ");
            for (byte i = 0; i < length; i++)
            {
                Serial.print(rxBuf[i], HEX);
                Serial.print(" ");
            }
            Serial.print("\n");

            unsigned long controllerId = rxId - 0x200;

            deriveFeedbackFields(rxBuf, &angle, &rpm, &amp, &temp);

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
        previousReceiveMillis = millis();
    }

    digitalWrite(13, HIGH);
}

/// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
/// @param milliAmperes 4つの-20000\~20000の電流値(mA)を格納した配列 (要素番号と速度コントローラIDが対応)
/// @param outTxBuf 結果の書き込み用配列
void scaleCurrentToRange(const signed long milliAmperes[4], byte outTxBuf[8])
{
    byte i;
    for (i = 0; i < 4; i++)
    {
        long milliAmpere = milliAmperes[i] * 16384 / 20000;
        byte upper = (milliAmpere >> 8) & 0xFF;
        byte lower = milliAmpere & 0xFF;
        outTxBuf[i * 2] = upper;
        outTxBuf[i * 2 + 1] = lower;
    }
}

/// @brief 速度コントローラから受け取ったデータから、フィードバック値を導出
/// @param rxBuf CANで受信した配列
/// @param outAngle ロータの角度(0°\~360°) (結果書き込み用)
/// @param outRpm 回転速度(rpm) (結果書き込み)
/// @param outAmp 実際のトルク電流(?) (結果書き込み用)
/// @param outTemp モータの温度(℃) (結果書き込み用)
void deriveFeedbackFields(const byte rxBuf[8], float *outAngle, unsigned int *outRpm, signed int *outAmp, unsigned int *outTemp)
{
    *outAngle = (float)(rxBuf[0] << 8 | rxBuf[1]) * 360.0f / 8191.0f;
    *outRpm = rxBuf[2] << 8 | rxBuf[3];
    *outAmp = rxBuf[4] << 8 | rxBuf[5];
    *outTemp = rxBuf[6];
}