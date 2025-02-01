#include <mcp_can.h>
#include <SPI.h>

#define CAN_ID (0x200)
#define CAN_SEND_INTERVAL (100)

MCP_CAN CAN(10); // CAN CS: pin 10

const int milli_amperes[4] = {10, 0, 0, 0};

void scaleCurrentToRange(const signed int milliAmperes[4], byte outTxBuf[8]);
void deriveFeedbackFields(const byte rxBuf[8], float *outAngle, unsigned int *outRpm, signed int *outAmp, unsigned int *outTemp);

/// @brief 前回のアップデート時間
long previousMillis;

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

    previousMillis = millis();
}

void loop()
{
    if ((millis() - previousMillis) > CAN_SEND_INTERVAL)
    {
        byte txBuf[8];
        scaleCurrentToRange(milli_amperes, txBuf);
        CAN.sendMsgBuf(CAN_ID, 0, 8, txBuf);
        Serial.println("Send ID: 0x200");
        previousMillis = millis();
    }

    if (CAN.checkReceive() == CAN_MSGAVAIL)
    {
        unsigned long rxId;
        byte length;
        byte rxBuf[8];
        CAN.readMsgBuf(&rxId, &length, rxBuf);
        Serial.print("Receive ID: ");
        Serial.print(rxId, HEX);
        Serial.print("Data: ");
        for (byte i = 0; i < length; i++)
        {
            Serial.print(rxBuf[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\n");

        unsigned long controllerId = rxId - 0x200;

        float angle;
        unsigned int rpm;
        signed int amp;
        unsigned int temp;
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

    digitalWrite(13, HIGH);
}

/// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
/// @param milliAmperes 4つの-20000\~20000の電流値(mA)を格納した配列 (要素番号と速度コントローラIDが対応)
/// @param outTxBuf 結果の書き込み用配列
void scaleCurrentToRange(const signed int milliAmperes[4], byte outTxBuf[8])
{
    int i;
    for (i = 0; i < 4; i++)
    {
        int milliAmpere = milliAmperes[i] * 16384 / 20000;
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