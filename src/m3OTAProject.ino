/**
  The plan is to use esp m3 as a master to control the
  two ATtiny 1616 mcu, where:
  at1 slave address: 0x12 (decimal value 18)
  at2 slave address 0x14 (decimal value 20)
  acccelerometer address is 0x19 LIS2DH12TR
  multiplexor 0x70
  color sensor: 0x10
  AT1 -
      Motor A/B
      RLED1, WLED1
      ENCODER_A/B/C
      TXD2/RXD2 (connected to m3 but no use) - this is improved by 2.25b
      RGB led.
  AT2 -
      IRSensor SEN1/2/3 (sensor 1 and 3 requires IR1/ Pin(16) to turn on led.
      LDR - SEN13/14
      Motor C/D
      RLED1, WLED1
      encoder:C/D
      Multiplexor
  i2c address:
      Decimal address:  16  | Hexa address:  0x10 - color sensors
      Decimal address:  18  | Hexa address:  0x12 -AT1
      Decimal address:  20  | Hexa address:  0x14 -AT2
      Decimal address:  25  | Hexa address:  0x19 -accelerotmeter
      Decimal address:  41  | Hexa address:  0x29 - tof
      Decimal address:  112  | Hexa address:  0x70 - multiplexor
  Software copyright 2024, tigo.robotics
 */
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ESP8266WebServer.h>
#include <StreamString.h>
#define FOR(I,N) for(int I=0;I<N;I++)
const char*apid = "TIGO_m3a_A__";
const char*pswd = "12345678";
ESP8266WebServer server(80);
#define AT1_SLAVE 0x12
#define AT2_SLAVE 0x14
#define MOTOR_MAX 120
#define TF_OFF
#define ACC_ON
#define CLR_ON
#define IRLED_OFF
#define AIR_SERIAL_PRINT_SENSORVAL 1

#ifdef ACC_ON
  #include "SparkFun_LIS2DH12.h"
  SPARKFUN_LIS2DH12 accel;
  float z_acc=0.0;
#endif
#ifdef TF_ON
  #include <VL53L0X.h>
  VL53L0X sensor; //0x29
  int head=0;
#endif
#ifdef CLR_ON
  #include "veml6040.h"
  VEML6040 RGBWSensor;
  int r1=0; int r2=0;
  int b1=0; int b2=0;
  int g1=0; int g2=0;
  int w1=0; int w2=0;
#endif
#ifdef IRLED_ON
  const int IR1 = 16; //this light up two fron IRleds.
  const int IR2 = 12;
  int ir1;
#endif

//function header 
void TCA9548A(uint8_t bus);
void to_MotorA(int dir, int speed);
void to_MotorD(int dir, int speed);
void to_RGB(long color);
void to_WLED1(char val);
void signalling(int);
void scanI2CDevice();
StreamString htmlBuffer;
StreamString outputBuffer;
char ipString[16];
int ldr1 = 0;
int ldr2 = 0;
int error_ldr, error_ldr_base;
int red_out = 0xFF;
int green_out = 0xFF;
int blue_out = 0xFF;
int red_out_last = 0xFF;
int green_out_last = 0xFF;
int blue_out_last = 0xFF;
void handleRoot() {
  IPAddress localIp = WiFi.localIP();
  sprintf(ipString,"%d.%d.%d.%d" ,localIp[0],localIp[1],localIp[2],localIp[3]);
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  //scanI2CDevice(); //only enable when needed.
  outputBuffer.replace("\n", "<br/>");
  htmlBuffer.clear();
  htmlBuffer.printf("<html><head><meta http-equiv='refresh' content='5'/></head>\
  <body>\
    <h1>esp m3 Data Output (Air Serial)</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <p>IP Adress: %s </p>\
    <p>Data: %s </p>\
  </body>\
</html>",
              hr, min % 60, sec % 60, ipString, outputBuffer.c_str());
  server.send(200, "text/html", htmlBuffer.c_str());
}

void setup() {
  htmlBuffer.reserve(200);  //approx 200 chars
  outputBuffer.reserve(200); //same
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apid, pswd);
  //IPAddress IP = WiFi.softAPIP();
  WiFi.begin("TIGO5G2.12", "abcdefab");
  // delay(1000);
  ArduinoOTA.begin();
  pinMode(2, OUTPUT);
  server.on("/", handleRoot);
  server.begin();
  Wire.setClock(400000); //set i2c fast mode.
  Wire.begin(4,14); //join i2c as master //sda4,scl14
  steerBot(0,0);
#ifdef TF_ON
  if (!sensor.init()) {
    FOR(k,3){
    signalling(30);
    delay(1000);
    }
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  #endif
  
  #ifdef ACC_ON
  if (!accel.begin()) {
    signalling(30);
    delay(100);
  }
  #endif
  #ifdef CLR_ON
  //switch to first clr sensr
  TCA9548A(0);
  delay(100);
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(100);
  FOR(i,5){
    if(!RGBWSensor.begin()) {
      signalling(30);
      delay(1000);
    }
  }
  //switch to first clr sensr
  TCA9548A(1);
  delay(100);
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(100);
  FOR(i,5){
   if(!RGBWSensor.begin()) {
     signalling(30);
     delay(1000);
    }
  }
  #endif  
  #ifdef IRLED_ON
    pinMode(IR1, OUTPUT);
    pinMode(IR2, OUTPUT);
    digitalWrite(IR1, 1);
    digitalWrite(IR2, 0);
  #endif

  to_WLED1('B');
  to_WLED2('B');

  //remember LDR difference:
  // read_IR_N_LDR();
  // error_ldr_base = (ldr1 - ldr2);

}

/*=======================MainLoop====================================*/

long tmr1 = 0;
bool botRun = 1;
void loop() {
  server.handleClient(); //use this to enable airSerial
  ArduinoOTA.handle(); //required for ota.
  
  //here we do our regular jobs:
  //signalling(100);
  // to_WLED1('A');
  // to_WLED2('A');
  //to_RGB(0xFFBBFF);
  //steerBot(0,0);
  read_IR_N_LDR();
  #ifdef IR_LEN_ON
  if (ir1 >80 ){
    blue_out = 0xF0;
    // to_WLED1('A');
    // to_WLED2('A');
  }else{
    blue_out = 0xFF;
    // to_WLED1('B');
    // to_WLED2('B');
  } 
  #endif
  #ifdef TF_ON
  head=sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) FOR(k,3)signalling(50);
  if(head > 200){
    to_WLED1('A');
    to_WLED2('A');
  }else{
    to_WLED1('B');
    to_WLED2('B');
  }
  //to_Int(head);
  #endif


  #ifdef ACC_ON
  z_acc = accel.getZ();
  if (z_acc < 0){
    red_out = 0xF0;
    green_out = 0xFF;
    //steerBot(abs(z_acc/10.0f),100);
    botRun = 1;
  }else{
    red_out = 0xFF;
    green_out = 0xF0;
    botRun = 0;
  }
  
  #endif
  
 #ifdef CLR_ON
  TCA9548A(0);
  // r1 = RGBWSensor.getRed();
  // g1 = RGBWSensor.getGreen();
  // b1 = RGBWSensor.getBlue();
  w1 = reget_white();
  TCA9548A(1);
  // r2 = RGBWSensor.getRed();
  // g2 = RGBWSensor.getGreen();
  // b2 = RGBWSensor.getBlue();
  w2 = reget_white();
  int error = (w2 - w1)*3;
  if (error < -100)error = -100;
  if (error > 100)error = 100;
  #ifdef AIR_SERIAL_PRINT_SENSORVAL
  outputBuffer.printf("w1 %d w2 %d ERROR: %d\n", w1, w2, error);
  #endif
  
  //alternative: use LDR as line sensor
  // error_ldr = (ldr1 - ldr2) - error_ldr_base ;
  //outputBuffer.printf("ERROR_LDR: %d\n",error_ldr);
  if(botRun==1){
    steerBot(error, 90);
    // steerBot(error_ldr, 50);
  }else{
    steerBot(0, 0);
  }
  #endif
  //update RGB 
  
  if(red_out_last   != red_out   || 
     blue_out_last  != blue_out  || 
     green_out_last != green_out    )
     to_RGB(red_out<<16 & 0xFF0000 | green_out<<8 & 0xFF00 | blue_out & 0xFF);
  red_out_last = red_out;
  blue_out_last = blue_out;
  green_out_last = green_out;
  //end of loop jobs
  outputBuffer.clear();
}



//This is the Multiplexor control
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// SENDING to COLOR RGB
void to_RGB(long color){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('R');
  Wire.write('G');
  Wire.write('B');//padding byte
  Wire.write(color>>16 & 0xFF); //R
  Wire.write(color>>8 & 0xFF); //G
  Wire.write(color & 0xFF); //B
  // Wire.write('E'); 
  Wire.endTransmission(); 
}

void to_MotorA(int dir, int speed){
  //control motor

  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('M');//0
  Wire.write('A');//1
  Wire.write((char)dir); //2 --> 1 or -1 to drive
  
  Wire.write((char)speed>15?speed-15:speed); //3 1 to 127
  // Wire.write('E');
  Wire.endTransmission(); 
}

void to_MotorD(int dir, int speed){
  //control motor
  Wire.beginTransmission(AT2_SLAVE); 
  Wire.write('M');//0
  Wire.write('D');//1
  Wire.write((char)dir); //2 --> 1 or -1 to drive
  Wire.write((char)speed); //3 1 to 127
  // Wire.write('E');
  Wire.endTransmission(); 
}

//sending 0 as string ?
void to_WLED1(char val){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('W');
  Wire.write('L');
  Wire.write('1');//padding byte
  Wire.write((char)val);
  //Wire.write('E');//padding byte
  Wire.endTransmission(); 
}

//sending 0 as string ?
void to_WLED2(char val){
  Wire.beginTransmission(AT2_SLAVE); 
  Wire.write('W');
  Wire.write('L');
  Wire.write('2');//padding byte
  Wire.write((char)val);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

//read from IR
void read_IR_N_LDR(){
  Wire.requestFrom(AT2_SLAVE, 8);    // request x bytes from sensor
  byte x3 = Wire.read();
  byte x2 = Wire.read();
  byte x1 = Wire.read();
  byte x0 = Wire.read();
  //--------extracting Light Dependent Resistor signal------------
  ldr1 = (x3 & 0xFF) << 8 | (x2 & 0xFF);
  ldr2 = (x1 & 0xFF) << 8 | (x0 & 0xFF);
  byte y3 = Wire.read(); 
  byte y2 = Wire.read();
  byte y1 = Wire.read();//no use
  byte y0 = Wire.read();//no use
  //--------extracting Infrared Receiver signal------------
  #ifdef AIR_SERIAL_PRINT_SENSORVAL  
  #ifdef IR_LED_ON
  ir1 = (y3 & 0xFF) << 8 | (y2 & 0xFF);
  //int ir2 = (y1 & 0xFF) << 8 | (y0 & 0xFF); //ir doesn't work.
  outputBuffer.printf("IR1: %d\n",ir1);
  #endif
  outputBuffer.printf("LDR1: %d, LDR2: %d \n",ldr1, ldr2);
  #endif
}

void signalling(int delaytime) {
  // Blink the LED as a signal
  for (int i = 0; i < 3; i++) {
    digitalWrite(2, HIGH);
    delay(delaytime);
    digitalWrite(2, LOW);
    delay(delaytime);
  }
}

void scanI2CDevice(){
  byte error, address;
  int nDevices;
  outputBuffer.clear();
  outputBuffer.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      outputBuffer.print("I2C device found at address 0x");
      if (address<16) outputBuffer.print("0");
      outputBuffer.print(address,HEX);
      outputBuffer.println("  !");
      nDevices++;
    }else if (error==4){
      outputBuffer.print("Unknown error at address 0x");
      if (address<16) outputBuffer.print("0");
      outputBuffer.println(address,HEX);
    }
  }
  if (nDevices == 0) outputBuffer.println("No I2C devices found\n");
  else outputBuffer.println("done\n");
}

/**
 * range of the motor power is actually from 70 to 120. 
 * so if user input something like a power of 1 percent 
 * we still need to move the motor. 
 * so the real working range is not 0-120, is 1 to 50!
*/
#define BASESPEED 40
#define WORKINGRANGE 50
void steerBot(int steerVal, int power){
  if(power==0){
    to_MotorA(0,0);
    to_MotorD(0,0);
  }else if (steerVal < -50 && steerVal >= -100){
    to_MotorA(power>0? -1: 1, BASESPEED+(abs(steerVal)-50) * WORKINGRANGE / abs(power));
    to_MotorD(power>0?  1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100);
  }else if(steerVal < 0 && steerVal >= -50){
    to_MotorA(power>0?  1:-1, BASESPEED+(50-abs(steerVal)) * WORKINGRANGE / abs(power));
    to_MotorD(power>0?  1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100);
  }else if(steerVal > 0 && steerVal <= 50){
    to_MotorA(power>0?  1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100); 
    to_MotorD(power>0?  1:-1, BASESPEED+(50-abs(steerVal)) * WORKINGRANGE / abs(power)); 
  }else if(steerVal > 50 && steerVal <= 100){
    to_MotorA(power>0?  1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100);
    to_MotorD(power>0? -1: 1, BASESPEED+(abs(steerVal)-50) * WORKINGRANGE / abs(power));
  }else{
    to_MotorA(power>0?1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100);
    to_MotorD(power>0?1:-1, BASESPEED+abs(power)*WORKINGRANGE / 100);
  }
}

void debug_to_html(){
  outputBuffer.clear();
  outputBuffer.printf("Test:0x%02X", 0xFF);
}

int reget_white(){
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  return RGBWSensor.getWhite();
}