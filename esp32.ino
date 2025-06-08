#include "BluetoothSerial.h"

// Use to recieve data from bluetooth, used Serial Bluetooth Terminal
BluetoothSerial SerialBT;

/*
  Use UART1 to send data from Bluetooth

  Note:
    UART0: On-board Serial
    UART1: Customized Serial
    UART2: Customized Serial
*/
HardwareSerial sendToUNO(1);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_GROUP3");

  /*
    Note: 
      Baud = 9600 (Similar to Arduino UNO)
      RX = 4
      TX = 5

      SERIAL_8N1:
        - 8: 8-bit data each byte
        - N: No parity
        - 1: 1-bit data at the end ('\n')
  */
  sendToUNO.begin(9600, SERIAL_8N1, 4, 5);

  Serial.println("Initialize successfully");
}

/* Loop the program */
void loop() {
  if (SerialBT.available()) {
    // Read data from bluetooth util the end of the line
    String data = SerialBT.readStringUtil('\n');

    // Send data to UNO by .println()
    sendToUNO.println(data);
  }
}
