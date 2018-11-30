/*

  RS485_HalfDuplex.pde - example using ModbusMaster library to communicate
  with MNU Modbus Ultrasonic Sensor using a half-duplex RS485 transceiver.

  Library:: ModbusMaster
  Author:: Robert Carle

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
#define MAX485_DE      3
#define MAX485_RE_NEG  2

#define MAX485_TX 11
#define MAX485_RX 10

// instantiate ModbusMaster object
ModbusMaster node;

// create software serial port
SoftwareSerial modbusSerial(MAX485_RX, MAX485_TX); //RX, TX

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus RTU Ultrasonic communication runs at 9600 baud
  modbusSerial.begin(9600);
  Serial.begin(9600);
 
  // Modbus slave ID 1
  node.begin(1, modbusSerial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

typedef union {
  uint16_t dblBytes[2];
  uint32_t bigNum;
} UINT32_ARRAY;


void loop()
{
  readTest();
}

void readTest() {
  uint8_t result;
  uint16_t data;
  
  // Read the register at address FC(0x03) Register(40420) Address(419) HEX(1A3) (Temperature Compensation)
  Serial.println("Reading temperature compensation register...");
  result = node.readHoldingRegisters(419, 1);
  
  if (result == node.ku8MBSuccess) {  
    Serial.print("Temperature Compensation Status: ");
      Serial.println(node.getResponseBuffer(0));
  } else {
    Serial.println("Failed to read temperature compensation register!");
    Serial.print("Result: ");Serial.println(result);
  }

  delay(1000);
  
  // Read 11 registers starting at FC(0x04) Register(30299) Address(298) HEX(12A)
  Serial.println("Reading all input registers...");
  result = node.readInputRegisters(298, 11);

  if (result == node.ku8MBSuccess) {
    Serial.print("ModelType: ");
    Serial.println(node.getResponseBuffer(0));
    
    Serial.print("Raw Distance/Level Reading (in mm, unsigned): ");
    Serial.println(node.getResponseBuffer(1)); 
    
    Serial.print("Temperature Reading (in C, signed): ");
    Serial.println(node.getResponseBuffer(3));

    Serial.print("Calculated (raw): ");
    UINT32_ARRAY val;
    val.dblBytes[0] = node.getResponseBuffer(4);
    val.dblBytes[1] = node.getResponseBuffer(5);
    Serial.println(val.bigNum);

    Serial.print("Version: ");
    data = node.getResponseBuffer(8);
    Serial.println(highByte(data));

    Serial.print("Signal Strength: ");
    Serial.println(lowByte(data));

    Serial.print("Trip 1 Alarm: ");
    data = node.getResponseBuffer(10);
    Serial.println(highByte(data));

    Serial.print("Trip 1 Status: ");
    Serial.println(lowByte(data));

    Serial.print("Trip 2 Alarm: ");
    data = node.getResponseBuffer(11);
    Serial.println(highByte(data));

    Serial.print("Trip 2 Status: ");
    Serial.println(lowByte(data));
  
  } else {
    Serial.println("Failed to read input registers...");
        Serial.print("Result: ");Serial.println(result);
  }
  
  delay(1000);
}

