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
const char*apid = "TIGO_m3a_A8";
const char*pswd = "12345678";
ESP8266WebServer server(80);
#define AT1_SLAVE 0x12
#define AT2_SLAVE 0x14
#define MOTOR_MAX 120
#define TF_OFF
#define ACC_ON
#define CLR_OFF

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
  int red=0; int mred=0;
  int blue=0; int mblue=0; 
  int green=0; int mgreen=0;
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
    if(!RGBWSensor.begin()) {
      FOR(i,5){
        signalling(30);
        delay(200);
      }
    }
  #endif

  #ifdef CLR_ON
  //switch to first clr sensr
  TCA9548A(0);
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(500);
  FOR(i,5){
    if(!RGBWSensor.begin()) {
      signalling(30);
      delay(1000);
    }
  }
  //switch to first clr sensr
  TCA9548A(1);
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(500);
  FOR(i,5){
   if(!RGBWSensor.begin()) {
     signalling(30);
     delay(1000);
    }
  }
  #endif  
}

void loop() {
  server.handleClient(); //use this to enable airSerial
  ArduinoOTA.handle(); //required for ota.
  //here we do our regular jobs:
  //signalling(100);
  to_WLED1('A');
  to_WLED2('A');
  //to_RGB(0xFFBBFF);
  //steerBot(0,0);
  

  #ifdef TF_ON
  head=sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) FOR(k,3)signalling(50);
  if(head > 300){
    digitalWrite(WLED2,1); //turn on
    to_WLED1('A');
  }else{
    digitalWrite(WLED2,0);
    to_WLED1('B');
  }
  //to_Int(head);
  #endif


  #ifdef ACC_ON
  z_acc = accel.getZ();
  if (z_acc < 0){
    to_RGB(0x00FFFF);
    steerBot(0,100);
  }else{
    to_RGB(0xFF00FF);
    steerBot(0,0);
  }
  #endif
  
 #ifdef CLR_ON
  TCA9548A(0);
  r1 = RGBWSensor.getRed();
  g1 = RGBWSensor.getGreen();
  b1 = RGBWSensor.getBlue();
  w1 = RGBWSensor.getWhite();
  #endif
  
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
  Wire.write((char)speed); //3 1 to 127
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
void steerBot(int steerVal, int power){
  if(power==0){
    to_MotorA(0,0);
    to_MotorD(0,0);
  }else if (steerVal < -50 && steerVal >= -100){
    to_MotorA(power>0?-1:1, 70+(abs(steerVal)-50) * MOTOR_MAX * 833  / abs(power) / 1000);
    to_MotorD(power>0?1:-1, abs(power));
  }else if(steerVal < 0 && steerVal >= -50){
    to_MotorA(power>0?-1:1, 70+(50-abs(steerVal)) * MOTOR_MAX * 833  / abs(power) / 1000);
    to_MotorD(power>0?1:-1, abs(power));
  }else if(steerVal > 0 && steerVal <= 50){
    to_MotorA(power>0?1:-1, abs(power));
    to_MotorD(power>0?-1:1, 70+(50-abs(steerVal)) * MOTOR_MAX * 833  / abs(power) / 1000);
  }else if(steerVal > 50 && steerVal <= 100){
    to_MotorA(power>0?1:-1, abs(power));
    to_MotorD(power>0?-1:1, 70+(abs(steerVal)-50) * MOTOR_MAX * 833  / abs(power) / 1000);
  }else{
    to_MotorA(power>0?1:-1, abs(power));
    to_MotorD(power>0?1:-1, abs(power));
  }
}

void debug_to_html(){
  outputBuffer.clear();
  outputBuffer.printf("Test:0x%02X", 0xFF);
}