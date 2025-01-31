#include <mcp_can.h>
#include <SPI.h>

byte tx_buf[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
long previous_millis;

MCP_CAN CAN(10); // CAN CS: pin 10

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

    previous_millis = millis();
}

void loop()
{
    static int count = 0;
    if (millis() - previous_millis > 100)
    { // Period: 100ms
        CAN.sendMsgBuf(0x200, 0, 8, tx_buf);
        Serial.println("Send ID: 0x200");
        previous_millis = millis();
    }

    if (CAN.checkReceive() == CAN_MSGAVAIL)
    {
        unsigned long rx_id;
        byte length;
        byte rx_buf[8];
        CAN.readMsgBuf(&rx_id, &length, rx_buf);
        Serial.print("Recive ID: ");
        Serial.print(rx_id, HEX);
        Serial.print("Data: ");
        for (byte i = 0; i < length; i++)
        {
            Serial.print(rx_buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    digitalWrite(13, HIGH);
}
