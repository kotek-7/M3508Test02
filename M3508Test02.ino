#include <mcp_can.h>
#include <SPI.h>

#define CAN_ID (0x200)
#define CAN_SEND_INTERVAL (100)

MCP_CAN CAN(10); // CAN CS: pin 10

const int milli_amperes[4] = {10, 0, 0, 0};

void scaleCurrentToRange(signed int milliAmperes[4], byte outTxBuf[8]);

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
        Serial.println();

    }

    digitalWrite(13, HIGH);
}

/// -20000mA\~20000mAの4要素の配列を、CANで速度コントローラに送信する8バイトの配列に変換
/// @param milliAmperes -20000\~20000の電流値(mA)を持つの長さ4の配列. 要素番号と速度コントローラのIDが対応してる
/// @param outTxBuf 結果の書き込み用配列
void scaleCurrentToRange(signed int milliAmperes[4], byte outTxBuf[8])
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