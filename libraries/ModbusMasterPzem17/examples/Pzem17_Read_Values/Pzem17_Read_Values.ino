#include <ModbusMasterPzem017.h>
  static uint8_t pzemSlaveAddr = 0x01; //PZem Address
    static uint16_t NewshuntAddr = 0x0002;      // Declare your external shunt value. Default is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
      ModbusMaster node;
        float PZEMVoltage =0;
        float PZEMCurrent =0;
        float PZEMPower =0;
        float PZEMEnergy=0;

void setup() 
{
  Serial.begin(115200);
    Serial2.begin(9600,SERIAL_8N2);
      setShunt(pzemSlaveAddr);
        node.begin(pzemSlaveAddr, Serial2);
          delay(1000);
}

void loop() {
  uint8_t result;
    result = node.readInputRegisters(0x0000, 6);
      if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
          PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;
          PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;
        tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); // get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit 
          PZEMPower = tempdouble / 10.0; //Divide the value by 10 to get actual power value (as per manual)
        tempdouble =  (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);  //get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit
          PZEMEnergy = tempdouble;
            Serial.print(PZEMVoltage, 1); //Print Voltage value on Serial Monitor with 1 decimal*/
            Serial.print("V   ");
            Serial.print(PZEMCurrent, 3); Serial.print("A   ");
            Serial.print(PZEMPower, 1); Serial.print("W  ");
            Serial.print(PZEMEnergy, 0); Serial.print("Wh  ");
              Serial.println();
    } else { Serial.println("Failed to read modbus");}
      delay(5000);
} //Loop Ends

void setShunt(uint8_t slaveAddr) {
  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003;                                                         /* change shunt register address command code */
  
  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);                                                         // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));
      
  Serial.println("Change shunt address");
  Serial2.write(slaveAddr); //these whole process code sequence refer to manual
  Serial2.write(SlaveParameter);
  Serial2.write(highByte(registerAddress));
  Serial2.write(lowByte(registerAddress));
  Serial2.write(highByte(NewshuntAddr));
  Serial2.write(lowByte(NewshuntAddr));
  Serial2.write(lowByte(u16CRC));
  Serial2.write(highByte(u16CRC));
    delay(10); delay(100);
    while (Serial2.available()) {
      Serial.print(char(Serial2.read()), HEX); //Prints the response and display on Serial Monitor (Serial)
      Serial.print(" ");
   }
} //setShunt Ends
