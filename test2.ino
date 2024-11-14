#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050_tockn.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_MPU6050 mpu;
MPU6050 mpu6050(Wire);
Adafruit_SSD1306 OLED(-1);

///////ตั้งค่าพอร์ต Analog /////////////

#define A0 500 // ค่ากลางเซนเซอร์ A0 
#define A1 450 // ค่ากลางเซนเซอร์ A1 
#define A2 450 // ค่ากลางเซนเซอร์ A2 
#define A3 450 // ค่ากลางเซนเซอร์ A3
#define A4 450 // ค่ากลางเซนเซอร์ A4
#define A5 450 // ค่ากลางเซนเซอร์ A5
#define A6 450 // ค่ากลางเซนเซอร์ A6
#define A7 450 // ค่ากลางเซนเซอร์ A7


///////////////////////////////////////////////   

///////////ตั้งค่าปุ่มกด///////////////////
//////////Sound buzzer///////////
int snd = 52;
//////// set io ///////////////////
int button =  53; ///button
int buttonL =  33; ///buttonL
int buttonR =  34; ///buttonR
int tk =  35;
int k_nob = 15 ; // K-nob
int ss1 = 24;  // servo ch1
int ss2 = 25;  // servo ch2
int ss3 = 26;  // servo ch3
Servo sv1;
Servo sv2;
Servo sv3;

////////////ตั้งค่าพอร์ตมอเตอร์////////////////////
#define DL1  9  // กำหนดสัญญาณดิจิตอลซ้ายที่ 1 พอร์ต 9
#define DL2  8  // กำหนดสัญญาณดิจิตอลซ้ายที่ 2 พอร์ต 8
#define PWML 2  /// กำหนดสัญญาณ PWM ซ้ายพอร์ต 2
//////////////////////////////////
#define DR1  11  /// กำหนดสัญญาณดิจิตอลขวาที่ 1 พอร์ต 11
#define DR2  10  /// กำหนดสัญญาณดิจิตอลขวาที่ 2 พอร์ต 10
#define PWMR 3 /// กำหนดสัญญาณ PWM ขวาพอร์ต 3
/////////////////////////////////////////////////
/////// ช่อง B //////////////////////////////////
#define DL1b  13  // กำหนดสัญญาณดิจิตอลซ้ายที่ 1 พอร์ต 9
#define DL2b  12  // กำหนดสัญญาณดิจิตอลซ้ายที่ 2 พอร์ต 8
#define PWMLb 4  /// กำหนดสัญญาณ PWM ซ้ายพอร์ต 2
//////////////////////////////////
#define DR1b  15  /// กำหนดสัญญาณดิจิตอลขวาที่ 1 พอร์ต 11
#define DR2b  14  /// กำหนดสัญญาณดิจิตอลขวาที่ 2 พอร์ต 10
#define PWMRb 5 /// กำหนดสัญญาณ PWM ขวาพอร์ต 3
/////// ช่อง c //////////////////////////////////
#define DL1c  17  // กำหนดสัญญาณดิจิตอลซ้ายที่ 1 พอร์ต 9
#define DL2c  16  // กำหนดสัญญาณดิจิตอลซ้ายที่ 2 พอร์ต 8
#define PWMLc 6  /// กำหนดสัญญาณ PWM ซ้ายพอร์ต 2
//////////////////////////////////
#define DR1c  23  /// กำหนดสัญญาณดิจิตอลขวาที่ 1 พอร์ต 11
#define DR2c  22  /// กำหนดสัญญาณดิจิตอลขวาที่ 2 พอร์ต 10
#define PWMRc 7 /// กำหนดสัญญาณ PWM ขวาพอร์ต 3
///////////////////////////////////////////////////////////////////////

int s0 ;
int s1 ;
int s2 ;
int s3 ;
int s4 ;
int s5 ;
int s6 ;
int s7 ;

int rs ;
int gs ;
int bs ;
int ys ;

int i ;

uint16_t r, g, b, c, colorTemp, lux;

///////////////////////////////////////////////////////////////////////////////
//////////////ตั้งค่า servo motor สำหรับฟังก์ชั่นต่างๆ//////////////

int set_sv1 = 80 ;  /* set ค่า servo motor ตัวที่ 1        */
int set_sv2 = 90 ;  /* set ค่า servo motor ตัวที่ 2        */
int set_sv3 = 0 ;  /* set ค่า servo motor ตัวที่ 3        */

int svR = 0   ;  /* องศา servo motor ยิงลูกบาศก์สีแดง        */
int svG = 180 ;  /* องศา servo motor ยิงลูกบาศก์สีเขียว       */
int svB = 0   ;  /* องศา servo motor ยิงลูกบาศก์น้ำเงิน       */
int svY = 180 ;  /* องศา servo motor ยิงลูกบาศก์สีเหลือง       */

int set_end  = 90 ;  /* องศา servo motor ยกธง        */
/*-----------------------------------------------------*/

///////////////////////////////////////////////////////////////////////////////
int sleep = 80 ; // ระยะเวลาให้หุ่นยนต์หยุด

int count = 80 ; // เดินตรงไปข้างหน้าตามจำนวนนับ
float Cck = 1.55 ; // ระยะเช็คสี
int Bypass = 450 ; // ระยะเวลาข้ามตะเกียบ & ขึ้นสะพาน
int spL = 100 ; // ความเร็ว motor ซ้าย
int spR = 100 ; // ความเร็ว motor ขวา

int Back = 165 ; // ระยะเวลาถอยหลัง
int spLB = 80 ; // ความเร็วถอยหลัง motor ซ้าย
int spRB = 80 ; // ความเร็วถอยหลัง motor ขวา

int SLb = 165 ; /// ระยะเวลาสไลด์ออก
int spSLB = 90 ; // ความเร็วมอเตอร์ซ้าย
int spSRB = 90 ; // ความเร็วมอเตอร์ขวา


int Lsp = 100 ; int gyroL = 82u
//ความเร็วเลี้ยวซ้าย  //องศา gyro ในการเลี้ยวซ้าย

int Rsp = 100 ; int gyroR = 79 ;
//ความเร็วเลี้ยวขวา  //องศา gyro ในการเลี้ยวขวา


int Tb = 100 ;           int gyroT = 180 ;
//ความเร็วในการหมุนกลับหลัง  //องศา gyro ในการการหมุนกลับหลัง

int tkml = 210 ;
int tkmr = 255 ;

int TempY = 2058 ; // ค่า color Temp ของสีเหลือง โดยหาได้จาก menu2
int TempB = 3555  ; // ค่า color Temp ของสีดำ โดยหาได้จาก menu2



void stop (){ while(1){RUN4(0,0);delay(100);
/////////////////ใส่เสียงตอนจบ
  tone(snd, 660, 100);delay(150);
  tone(snd, 660, 100);delay(300);
  tone(snd, 660, 100);delay(300);
  tone(snd, 510, 100);delay(100);
  tone(snd, 660, 100);delay(300);
  tone(snd, 770, 100);delay(550);
  tone(snd, 380, 100);delay(575);
  tone(snd, 510, 100);delay(450);
  tone(snd, 380, 100);delay(400);
  tone(snd, 320, 100);delay(500);
  tone(snd, 440, 100);delay(300);
  tone(snd, 480, 80);delay(330);
  tone(snd, 450, 100);delay(150);
  tone(snd, 430, 100);delay(300);
  tone(snd, 380, 100);delay(200);
  tone(snd, 660, 80);delay(200);
  tone(snd, 760, 50);delay(150);
  tone(snd, 860, 100);delay(300);
  tone(snd, 700, 80);delay(150);
  tone(snd, 760, 50);delay(350);
  tone(snd, 660, 80);delay(300);
  tone(snd, 520, 80);delay(150);
  tone(snd, 580, 80);delay(150);
  tone(snd, 480, 80);delay(500);
  tone(snd, 510, 100);delay(450);
  tone(snd, 380, 100);delay(400);
  tone(snd, 320, 100);delay(500);
  tone(snd, 440, 100);delay(300);
  tone(snd, 480, 80);delay(330);
  tone(snd, 450, 100);delay(150);
  tone(snd, 430, 100);delay(300);
  tone(snd, 380, 100);delay(200);
  tone(snd, 660, 80);delay(200);
  tone(snd, 760, 50);delay(150);
  tone(snd, 860, 100);delay(300);
  tone(snd, 700, 80);delay(150);
  tone(snd, 760, 50);delay(350);
  tone(snd, 660, 80);delay(300);
  tone(snd, 520, 80);delay(150);
  tone(snd, 580, 80);delay(150);
  tone(snd, 480, 80);delay(500);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 150);delay(300);
  tone(snd, 380, 100);delay(150);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(100);
  tone(snd, 570, 100);delay(220);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 200);delay(300);
  tone(snd, 1020, 80);delay(300);
  tone(snd, 1020, 80);delay(150);
  tone(snd, 1020, 80);delay(300);
  tone(snd, 380, 100);delay(300);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 150);delay(300);
  tone(snd, 380, 100);delay(150);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(100);
  tone(snd, 570, 100);delay(420);
  tone(snd, 585, 100);delay(450);
  tone(snd, 550, 100);delay(420);
  tone(snd, 500, 100);delay(360);
  tone(snd, 380, 100);delay(300);
  tone(snd, 500, 100);delay(300);
  tone(snd, 500, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 150);delay(300);
  tone(snd, 380, 100);delay(150);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(100);
  tone(snd, 570, 100);delay(220);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 200);delay(300);
  tone(snd, 1020, 80);delay(300);
  tone(snd, 1020, 80);delay(150);
  tone(snd, 1020, 80);delay(300);
  tone(snd, 380, 100);delay(300);
  tone(snd, 500, 100);delay(300);
  tone(snd, 760, 100);delay(100);
  tone(snd, 720, 100);delay(150);
  tone(snd, 680, 100);delay(150);
  tone(snd, 620, 150);delay(300);
  tone(snd, 650, 150);delay(300);
  tone(snd, 380, 100);delay(150);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 430, 100);delay(150);
  tone(snd, 500, 100);delay(100);
  tone(snd, 570, 100);delay(420);
  tone(snd, 585, 100);delay(450);
  tone(snd, 550, 100);delay(420);
  tone(9, 500, 100);delay(360);
  tone(snd, 380, 100);delay(300);
  tone(snd, 500, 100);delay(300);
  tone(snd, 500, 100);delay(150);
  tone(snd, 500, 100);delay(300);
  tone(snd, 500, 60);delay(150);
  tone(snd, 500, 80);delay(300);
  tone(snd, 500, 60);delay(350);
  tone(snd, 500, 80);delay(150);
  tone(snd, 580, 80);delay(350);
  tone(snd, 660, 80);delay(150);
  tone(snd, 500, 80);delay(300);
  tone(snd, 430, 80);delay(150);
  tone(snd, 380, 80);delay(600);
  tone(snd, 500, 60);delay(150);
  tone(snd, 500, 80);delay(300);
  tone(snd, 500, 60);delay(350);
  tone(snd, 500, 80);delay(150);
  tone(snd, 580, 80);delay(150);
  tone(snd, 660, 80);delay(550);
  tone(snd, 870, 80);delay(325);
  tone(snd, 760, 80);delay(600);
  tone(snd, 500, 60);delay(150);
  tone(snd, 500, 80);delay(300);
  tone(snd, 500, 60); delay(350);
  tone(snd, 500, 80); delay(150);
  tone(snd, 580, 80); delay(350);
  tone(snd, 660, 80); delay(150);
  tone(snd, 500, 80); delay(300);
  tone(snd, 430, 80); delay(150);
  tone(snd, 380, 80); delay(600);
  tone(snd, 660, 100); delay(150);
  tone(snd, 660, 100); delay(300);
  tone(snd, 660, 100); delay(300);
  tone(snd, 510, 100); delay(100);
  tone(snd, 660, 100); delay(300);
  tone(snd, 770, 100); delay(550);
  tone(snd, 380, 100); delay(575);
/////////////////
} }
///////////////////////////////////////////////////////////////////////////////

void analogs()
{
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    int s4 = analogRead(4);
    int s5 = analogRead(5);
    int s6 = analogRead(6);
    int s7 = analogRead(7);
}
//////////////////////////////////////////////
void setup() {
  Wire.begin();
  tcs.begin();
  mpu6050.begin();
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C); // address i2c 0x3C (for the 128x64)
  pinMode(button , INPUT);
  pinMode(buttonL , INPUT);
  pinMode(buttonR , INPUT);
  pinMode(tk , INPUT);
  pinMode(DL1, OUTPUT);
  pinMode(DL2, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(DR2, OUTPUT);
  pinMode(PWMR, OUTPUT);
  ////////////////////////////
  pinMode(DL1b, OUTPUT);
  pinMode(DL2b, OUTPUT);
  pinMode(PWMLb, OUTPUT);
  pinMode(DR1b, OUTPUT);
  pinMode(DR2b, OUTPUT);
  pinMode(PWMRb, OUTPUT);
  //////////////////////
  pinMode(DL1c, OUTPUT);
  pinMode(DL2c, OUTPUT);
  pinMode(PWMLc, OUTPUT);
  pinMode(DR1c, OUTPUT);
  pinMode(DR2c, OUTPUT);
  pinMode(PWMRc, OUTPUT);
  //////////////////////////// 
  ////// Servo set ////////////
  sv1.attach(ss1);
  sv2.attach(ss2);
  sv3.attach(ss3);

  svset();
}

void beep(){tone(snd,1200,100);}
void loop() {
  if (rs and bs and gs and ys >= 1){end();}
  int sw = digitalRead(button);     // set sw = button
  int nob = analogRead(k_nob);          // variable nob = k_nob
  int menu = map(nob, 0, 1023, 0, 12); // mab nob 0-4095 to 0-12
  OLED.clearDisplay();              //clear oled
  OLED.setTextColor(WHITE, BLACK);  //set color
  OLED.setCursor(0, 0);       // set cursor position
  OLED.setTextSize(2);        // set text size 2 point
  OLED.print("     ");         // space text
  OLED.println(menu);        // display menu 0 to 12
  OLED.setTextSize(1);        //  set text size 1 point
  OLED.println("    LOTUS ARDUIBOT");     // display  LOTUS ARDUIBOT
  OLED.print("      ");                      // space text
  OLED.print(nob);                     // display nob
  OLED.println(" TEST");     // display TEST
  OLED.setTextSize(2);        //  set text size 2 point
  OLED.display();
  if ((sw == 0) and (menu == 0))
  { sensor();
  }
  if ((sw == 0) and (menu == 1))
  {
    menu1();
  }
  if ((sw == 0) and (menu == 2))
  {
    menu2();
  }
  if ((sw == 0) and (menu == 3))
  {
    menu3();
  }
  if ((sw == 0) and (menu == 4))
  {
    menu4();
  }
  if ((sw == 0) and (menu == 5))
  {
    menu5();
  }
  if ((sw == 0) and (menu == 6))
  {
    menu6();
  }
  if ((sw == 0) and (menu == 7))
  {
    menu7();
  }
  if ((sw == 0) and (menu == 8))
  {
    menu8();
  }
  if ((sw == 0) and (menu == 9))
  {
    menu9();
  }
  if ((sw == 0) and (menu == 10))
  {
    menu10();
  }
  if ((sw == 0) and (menu == 11))
  {
    menu11();
  }
  if ((sw == 0) and (menu == 12))
  {
    menu12();
  }
}

void sensor()
{
    mpu6050.begin();
    
    while (true) {

    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    int s4 = analogRead(4);
    int s5 = analogRead(5);
    int s6 = analogRead(6);
    int s7 = analogRead(7);
    
    mpu6050.update();
    float gyro = mpu6050.getAngleZ();
    OLED.clearDisplay();
    OLED.setTextColor(WHITE, BLACK);  //สีอักษรเป็นสีขาว ,พื้นหลังดำ
    OLED.setCursor(0, 0);       // เซตตำแหน่ง 0,0
    OLED.setTextSize(0.2);        // เซตขนาดอักษรมีขนาดเป็น 1
    OLED.print("       S0 = "); OLED.println(s0);  // แสดงค่าเซนเซอร์ S0
    OLED.print("       S1 = "); OLED.println(s1);  // แสดงค่าเซนเซอร์ S1
    OLED.print("       S2 = "); OLED.println(s2);  // แสดงค่าเซนเซอร์ S2
    OLED.print("       S3 = "); OLED.println(s3);  // แสดงค่าเซนเซอร์ S3
    OLED.print("       S4 = "); OLED.println(s4);  // แสดงค่าเซนเซอร์ S4
    OLED.print("       S5 = "); OLED.println(s5);  // แสดงค่าเซนเซอร์ S5
    OLED.print("       S6 = "); OLED.println(s6);  // แสดงค่าเซนเซอร์ S6
    OLED.print("       S7 = "); OLED.println(s7);  // แสดงค่าเซนเซอร์ S7
    OLED.println("       -------");
    OLED.print("     GYRO = ");  OLED.println(gyro); 
    OLED.println("       -------");
    OLED.display();
    delay(50);
  }
}

void Sensor_RGB()
{
  while(true){

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);    
  int avg = (r + g + b)/3;
  
  OLED.clearDisplay();
  OLED.setTextColor(WHITE,BLACK);   
  OLED.setCursor(0,0);   
  OLED.setTextSize(1);

  OLED.print("R: "); OLED.println(r, DEC);
  OLED.print("G: "); OLED.println(g, DEC);
  OLED.print("B: "); OLED.println(b, DEC);
  OLED.print("C: "); OLED.println(c, DEC);
  OLED.print("Lux: "); OLED.println(lux, DEC);
  OLED.print("colorTemp: "); OLED.println(colorTemp, DEC);

  if ((r > g) && (r > b) && (g > b) && (r > avg) && (b < avg) && (TempY+200 > colorTemp))
  {OLED.setTextSize(1);OLED.print("YELLOW");}

  else if ((r > g) && (r > b) && (r < lux))
  {OLED.setTextSize(1);OLED.print("RED");}
  
  else if ((r < g) && (r < b) && (g+10 > b) && (TempB+250 < colorTemp))
  {OLED.setTextSize(1);OLED.print("GREEN");}

  else if ((r < g) && (r < b) && (g > b) && (TempB+250 > colorTemp))
  {OLED.setTextSize(1);OLED.print("BLACK");}
  
  else if ((r < g) && (r < b) && (g < b))
  {OLED.setTextSize(1);OLED.print("BLUE");}

  OLED.display();
  
  }
}

void ShootRY()
{ 
  int sh = 0 ;
  int shC = 2 ;
   
  while(true){

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  int avg = (r + g + b)/3;

  if(sh >= shC){
    
  OLED.clearDisplay();
  OLED.setTextColor(WHITE,BLACK);   
  OLED.setCursor(0,0);   
  OLED.setTextSize(2);

if (rs and bs and gs and ys >= 1){end();break;}

if ((r > g) && (r > b) && (g > b) && (r > avg) && (b < avg) && (TempY+200 > colorTemp))
                                      {OLED.print("YELLOW");OLED.display();  if (ys == 0){ys = 1;delay(500);tone(3,1500,200);delay(300);menu6();menu6();menu6();  sv2.write(svY);  delay(500); sv2.write(set_sv2-20);delay(300);if (rs and bs and gs and ys >= 1){end();stop();break;}colorRY();break;}                                                             
                                                                             if (rs and bs and gs and ys >= 1){end();stop();break;}
                                                                             else if  (ys == 1){colorRY();break;}
                                                                             }
if ((r > g) && (r > b) && (r < lux))
                                      {OLED.print("RED");OLED.display();     if (rs == 0){rs = 1;delay(500);tone(3,1500,200);delay(300);menu6();menu6();menu6();  sv1.write(svR);  delay(500); sv1.write(set_sv1+20);delay(300);if (rs and bs and gs and ys >= 1){end();stop();break;}colorRY();break;}                                                        
                                                                             if (rs and bs and gs and ys >= 1){end();stop();break;}
                                                                             else if (rs == 1){colorRY();break;}  
                                                                             } 
else {break;}
  }
 sh++;}
}

void ShootGB()
{
  run(100,100);delay(200);run(0,0);delay(10);
  int sh = 0 ;
  int shC = 2 ;

  while(true){

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  int avg = (r + g + b)/3;

  if(sh >= shC){
    
  OLED.clearDisplay();
  OLED.setTextColor(WHITE,BLACK);   
  OLED.setCursor(0,0);   
  OLED.setTextSize(2);
                                                                                                                                              
if ((r < g) && (r < b) && (g > b) && (TempB+250 > colorTemp)){menu6();menu6();break;}
                                                                                                                                                
if ((r > g) && (r > b) && (r < lux)){break;}
                                                                                                                                               
if ((r < g) && (r < b) && (g+10 > b))
                                      {OLED.print("GREEN");OLED.display();    if (gs == 0){gs = 1;delay(500);tone(3,1500,200);delay(300);  sv1.write(svG);  delay(500); sv1.write(set_sv1-20);delay(300);if (rs and bs and gs and ys >= 1){end();stop();break;}colorGB();break;}                                                            
                                                                              if (rs and bs and gs and ys >= 1){end();stop();break;}
                                                                              else if (gs == 1){colorGB();break;}
                                                                             }                                                                                                                                          
if ((r < g) && (r < b) && (g < b))
                                      {OLED.print("BLUE");OLED.display();     if (bs == 0){bs = 1;delay(500);tone(3,1500,200);delay(300);  sv2.write(svB);  delay(500); sv2.write(set_sv2+20);delay(300);if (rs and bs and gs and ys >= 1){end();stop();break;}colorGB();break;}                                                 
                                                                              if (rs and bs and gs and ys >= 1){end();stop();break;}
                                                                              else if (bs == 1){colorGB();break;} 
                                                                             } 
else {break;}
  }
 sh++;}
}

void svset(){sv1.write(set_sv1);delay(500); sv2.write(set_sv2);delay(500); sv3.write(set_sv3);delay(500);}

void end(){sv3.write(set_end);delay(100);sv3.write(set_sv3);delay(100);sv3.write(set_end);delay(100);sv3.write(set_sv3);delay(100);sv3.write(set_end);delay(100);}

/////////////////////////////ฟังก์ชันเซอร์โว////////////////////////////////////////
void sv_knob() {
  while (true) {
    int vr = analogRead(15);  // กำหนดตัวแปรจำนวนเต็มอ่านค่าอนาล็อกที่พอร์ต 7
    int nob = map(vr, 0, 1023, 0, 180); // ทำการ map อัตราส่วนจากสัญญาณ analog 0-4095 เป็น 0-180
    OLED.clearDisplay();    // เคลียร์หน้าจอ oled
    OLED.setTextColor(WHITE, BLACK);  //สีอักษรเป็นสีขาว ,พื้นหลังดำ
    OLED.setCursor(0, 0);       // เซตตำแหน่ง 0,0
    OLED.setTextSize(2);        // เซตขนาดอักษรมีขนาดเป็น 2
    OLED.print("SV1 = ");     // พิมพ์คำว่า SV1 =
    OLED.println(nob);     // นำค่า nob มาแสดงใน oled
    OLED.print("SV2 = ");     // พิมพ์คำว่า SV2 =
    OLED.println(nob);     // นำค่า nob มาแสดงใน oled
    OLED.print("SV3 = ");     // พิมพ์คำว่า SV3 =
    OLED.println(nob);     // นำค่า nob มาแสดงใน oled
    OLED.print("----------");     // พิมพ์คำว่า SV3 =
    OLED.display();        // เปิดฟังก์ชันแสดงผล
    sv1.write(nob);        // สั่งเซอร์โวมอเตอร์ให้หมุนไปตามค่าองศาที่ทำการ nob ไว้
    sv2.write(nob);        // สั่งเซอร์โวมอเตอร์ให้หมุนไปตามค่าองศาที่ทำการ nob ไว้
    sv3.write(nob);        // สั่งเซอร์โวมอเตอร์ให้หมุนไปตามค่าองศาที่ทำการ nob ไว้
    delay(50);             // หน่วงเวลา 0.05 วินาที
  }
}

void P(int t)
{ 
  float ckColor = count/Cck;
  i = 0 ;
  while(1)
  {
    
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    int s4 = analogRead(4);
    int s5 = analogRead(5);
    int s6 = analogRead(6);
    int s7 = analogRead(7);

    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);
    int tk      = digitalRead(34);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}

    if (tk == 1) {while(1){RUN4(tkml,tkmr);delay(t);RUN4(0,0);delay(200);menu9();ckT();menu10();ckT();menu3();break;}} 
    /* code chopsticks */
    
    RUN4(spL,spR);delay(10);
    

    if ((s0 < A1) && (s5 < A5) && (ckColor > i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);menu6();menu5();ckT();menu6();ckT();menu7();}
    else if ((s1 < A1) && (ckColor > i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);menu6();menu5();ckT();menu6();ckT();menu7();}
    else if ((s2 < A2) && (ckColor > i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);menu6();menu5();ckT();menu6();ckT();menu7();}

    else if((s1 < A1) && (s2 < A2) && (ckColor < i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();}
    else if((s1 < A1) && (ckColor < i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();}
    else if((s2 < A2) && (ckColor < i)) {RUN4(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();}
    
    if (s0 < A0) {RUN4(150,-200);delay(20);}
    if (s3 < A3) {RUN4(-200,150);delay(20);}

    if (i >= count) {RUN4(0,0);delay(sleep);menu9();ckT();menu10();ckT();menu3();}
    
    i++;
  }
}

void PP(int t)
{ 
  float ckColor = count/Cck;
  i = 0 ;
  while(1)
  {
    
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    int s4 = analogRead(4);
    int s5 = analogRead(5);
    int s6 = analogRead(6);
    int s7 = analogRead(7);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);
    int tk      = digitalRead(34);
    
    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}

    if (tk == 1) {while(1){RUN4(tkml,tkmr);delay(t);RUN4(0,0);delay(200);menu4();ckT();menu6();ckT();menu3();break;}}

    RUN4(spL,spR);delay(10);
    
    
    if ((s1 < A1) && (s2 < A2) && (ckColor > i)) {run(0,0);delay(sleep);beep();set_robot(100,100);menu6();ShootRY();menu5();ckT();menu6();ckT();menu7();}
    else if ((s1 < A1) && (ckColor > i)) {run(0,0);delay(sleep);beep();set_robot(100,100);menu6();ShootRY();menu5();ckT();menu6();ckT();menu7();}
    else if ((s2 < A2) && (ckColor > i)) {run(0,0);delay(sleep);beep();set_robot(100,100);menu6();ShootRY();menu5();ckT();menu6();ckT();menu7();}
 
    else if ((s1 < A1) && (s2 < A2) && (ckColor < i)) {run(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();} 
    else if ((s1 < A1) && (ckColor < i)) {run(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();}
    else if ((s2 < A2) && (ckColor < i)) {run(0,0);delay(sleep);beep();set_robot(100,100);ShootGB();menu6();menu5();ckT();menu6();ckT();menu7();}

    if (s0 < A0) {RUN4(150,-200);delay(20);}
    if (s3 < A3) {RUN4(-200,150);delay(20);}
   
  
    if (i >= count) {RUN4(0,0);delay(sleep);menu9();ckT();menu10();ckT();menu3();}
    
    i++;
  }
}

void set_robotR(int L ,int R)
{ menu6();
  while(1)
  {
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}
    if (s0 < A0) {RUN4(-200,100);}
    if ((s0 < A0) && (s3 > A3)) {RUN4(-200,100);}
    if (s3 < A3) {RUN4(100,-200);}
    if ((s0 > A0) && (s3 < A3)) {RUN4(100,-200);}
    if ((s0 < A0) && (s3 < A3)) {RUN4(0,0);delay(sleep);break;}
    if ((s0 > A0) && (s3 > A3)) {RUN4(L,R);}
  }
}

void set_robotL(int L ,int R)
{ menu6();
  while(1)
  {
    int s0 = analogRead(0);
    int s3 = analogRead(3);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}
    if (s0 < A0) {RUN4(-200,100);}
    if ((s0 < A0) && (s3 > A3)) {RUN4(-200,100);}
    if (s3 < A3) {RUN4(100,-200);}
    if ((s0 > A0) && (s3 < A3)) {RUN4(100,-200);}
    if ((s0 < A0) && (s3 < A3)) {RUN4(0,0);delay(sleep);break;}
    if ((s0 > A0) && (s3 > A3)) {RUN4(L,R);}

  }
}

void set_robot(int L ,int R)
{ menu6();
  while(1)
  {
    int s0 = analogRead(0);
    int s3 = analogRead(3);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}
    if (s0 < A0) {RUN4(-200,100);}
    if ((s0 < A0) && (s3 > A3)) {RUN4(-200,100);}
    if (s3 < A3) {RUN4(100,-200);}
    if ((s0 > A0) && (s3 < A3)) {RUN4(100,-200);}
    if ((s0 < A0) && (s3 < A3)) {RUN4(0,0);delay(sleep);break;}
    if ((s0 > A0) && (s3 > A3)) {RUN4(L,R);}
  }
}

void set_robotF(int L ,int R)
{ menu6();
  while(1)
  {
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(4);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}
    if ((s0 < A0) && (s1 < A1)) ) {RUN4(100,-200)}
    if ((s3 < A3) ) {RUN4(-200,100)}
  /*if ((s3 < A3) && (s2 < A2)) {RUN4(100,-200);}
    if ((s0 > A0) && (s3 < A3)) {RUN4(100,-200);}*/
    if ((s0 < A0) &&  (s3 < A3) ) {RUN4(0,0);delay(sleep);break;}
    if ((s0 > A0) && (s3 > A3)) {RUN4(L,R);}
  }
}


void set_robotS(int L ,int R)
{ menu10();
  while(1)
  {
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s4 = analogRead(4);
    int s5 = analogRead(5);
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    if (buttonL == 1) {set_lr(spL,spR);}
    if (buttonR == 1) {set_lr(spL,spR);}
    if ((s0 < A0) ) {l1ARC(200,100)}
    if ((s4 < A4) ) {l2ARC(200,100)}
  /*if ((s3 < A3) && (s2 < A2)) {RUN4(100,-200);}
    if ((s0 > A0) && (s3 < A3)) {RUN4(100,-200);}*/
    if ((s0 < A0) &&  (s4 < A4 )) {RUN4(0,0);delay(sleep);break;}
    if ((s0 > A0) && (s3 > A3)) {slideL(L,R);}
  }
}

void set_lr(int L ,int R)
{ i = 0 ;
  while(1)
  {
    int buttonL = digitalRead(32);
    int buttonR = digitalRead(33);

    RUN4(L,R);
    if (buttonL == 1) {RUN4(-200,0);delay(1);}
    if (buttonR == 1) {RUN4(0,-200);delay(1);}

    if ((buttonL == 1) && (buttonR == 1)) {RUN4(0,0);delay(10);bypass();}
    if (i >= count) {RUN4(0,0);delay(sleep);bypass();}

    i++;
  }
}

void colorGB() {menu6();menu6();menu6();menu6();menu5();ckT();menu6();ckT();menu3();}
void colorRY() {menu6();menu6();menu6();menu6();menu5();ckT();menu6();ckT();menu3();}

void bypass() {menu5();ckT();menu6();ckT();menu3();}

void ckT()
{
  while(1)
  { 
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(2);
    int s3 = analogRead(3);
    
    if (s0 < A0) {menu6();break;}
    if (s1 < A1) {menu6();break;}
    if (s2 < A2) {menu6();break;}
    if (s3 < A3) {menu6();break;}
    break;
  }
}

void L(int m, int cp)
{ 
  mpu6050.begin();
  if (rs and bs and gs and ys >= 1){end();stop();}
  int GYRO = mpu6050.getAngleZ()+cp ;
  RUN4(-m,m);
  while(1)
  {
    int tk = digitalRead(34);
    mpu6050.update();
    
    if (mpu6050.getAngleZ() >  GYRO) {RUN4(0,0);delay(100);break;}
    if (tk == 1) {while(1){mpu6050.update();if (mpu6050.getAngleZ() >  GYRO) {RUN4(0,0);delay(100);break;}RUN4(100,100);delay(500);menu4();menu3();}
  }
}
}

void R(int m, int cp)
{ 
  mpu6050.begin();
  if (rs and bs and gs and ys >= 1){end();stop();}
  int GYRO = mpu6050.getAngleZ()-cp ;
  RUN4(m,-m);
  while(1)
  {
    int tk = digitalRead(34);
    mpu6050.update();
    
    if (mpu6050.getAngleZ() <  GYRO) {RUN4(0,0);delay(100);break;}
    if (tk == 1) {while(1){mpu6050.update();if (mpu6050.getAngleZ() <  GYRO) {RUN4(0,0);delay(100);break;}RUN4(100,100);delay(500);menu4();menu3();}
  }
}
}


void B(int L, int R ,int t){RUN4(-L,-R);delay(t);RUN4(0,0);delay(10);}
/* Silde */
void slideL(int l ,int r,int t ){runa(l,-r);runb(-l,r);delay(t);RUN4(0,0);delay(10);}
void slideR(int l ,int r,int t){runa(-l,r);runb(l,-r);delay(t);RUN4(0,0);delay(10);}
void slideL0(int l ,int r,){runa(l,-r);runb(-l,r);}
void slideR0(int l ,int r){runa(-l,r);runb(l,-r);}
/* LETERAL ARC*/
void l1ARC(int l ,int r ) {runa(l,-r);RUN4(0,0);delay(10);}
void l2ARC(int l ,int r ) {runb(l,-r);RUN4(0,0);delay(10);}
/*Turn back*/

void turnb(int m , int cp){ 
  mpu6050.begin();
  if (rs and bs and gs and ys >= 1){end();stop();}
  int GYRO = mpu6050.getAngleZ()+cp ;
  RUN4(m,-m);
  while(1)
  {
    int tk = digitalRead(34);
    mpu6050.update();
    
    if (mpu6050.getAngleZ() >  GYRO) {RUN4(0,0);delay(100);break;}
    if (tk == 1) {while(1){mpu6050.update();if (mpu6050.getAngleZ() >  GYRO) {RUN4(0,0);delay(100);break;}RUN4(100,100);delay(500);menu4();menu3();}
    // chopsticks 
  }
}
}

///////////////////////////////////////////////////////////////////
void RUN4(int l ,int r){runa(l,r);runb(l,r);}
///////////////////////////////////////////////////////////////////
void RUN(int ml , int mr){runa(ml,mr);runb(ml,mr);runc(ml,mr); } // คำสั่งควบคุมมอเตอร์ทั้ง 6 ช่อง
///////////////////////////////////////////////////////////////////
void run(int spr, int spl)   // ประกาศฟังก์ชัน run(กำลังมอเตอร์ซ้าาย,กำลังมอเตอร์ขวา);
{ if (spl > 0)
  {   
    digitalWrite(DL1, LOW);
    digitalWrite(DL2, HIGH);
    analogWrite(PWML, spl);
  }
else if (spl < 0)
  {
    digitalWrite(DL1, HIGH);
    digitalWrite(DL2, LOW);
    analogWrite(PWML, -spl);
  }      
else
  {             
    digitalWrite(DL1, HIGH);
    digitalWrite(DL2, HIGH);
    analogWrite(PWML, -255);
  }
///////////////////////////////////////////////
if (spr > 0)
  {     
    digitalWrite(DR1, LOW);
    digitalWrite(DR2, HIGH);
    analogWrite(PWMR, spr);
  }
else if (spr < 0)
  {
    digitalWrite(DR1, HIGH);
    digitalWrite(DR2, LOW);
    analogWrite(PWMR, -spr);
  }
else
  {
    digitalWrite(DR1, HIGH);
    digitalWrite(DR2, HIGH);
    analogWrite(PWMR, -255);
  }}
  void runa(int spra, int spla)   // ประกาศฟังก์ชัน run(กำลังมอเตอร์ซ้าาย,กำลังมอเตอร์ขวา);
{ if (spla > 0)
  {   
    digitalWrite(DL1, LOW);
    digitalWrite(DL2, HIGH);
    analogWrite(PWML, spla);
  }
else if (spla < 0)
  {
    digitalWrite(DL1, HIGH);
    digitalWrite(DL2, LOW);
    analogWrite(PWML, -spla);
  }      
else
  {             
    digitalWrite(DL1, HIGH);
    digitalWrite(DL2, HIGH);
    analogWrite(PWML, -255);
  }
///////////////////////////////////////////////
if (spra > 0)
  {     
    digitalWrite(DR1, LOW);
    digitalWrite(DR2, HIGH);
    analogWrite(PWMR, spra);
  }
else if (spra < 0)
  {
    digitalWrite(DR1, HIGH);
    digitalWrite(DR2, LOW);
    analogWrite(PWMR, -spra);
  }
else
  {
    digitalWrite(DR1, HIGH);
    digitalWrite(DR2, HIGH);
    analogWrite(PWMR, -255);
  }}


  
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
void runb(int sprb, int splb)   // ประกาศฟังก์ชัน run(กำลังมอเตอร์ซ้าาย,กำลังมอเตอร์ขวา);
{ if (splb > 0)
  {
  digitalWrite(DL1b, LOW);
  digitalWrite(DL2b, HIGH);
  analogWrite(PWMLb, splb);
  }
else if (splb < 0)
  {
    digitalWrite(DL1b, HIGH);
    digitalWrite(DL2b, LOW);
    analogWrite(PWMLb, -splb);
  }
else
  {
    digitalWrite(DL1b, HIGH);
    digitalWrite(DL2b, HIGH);
    analogWrite(PWMLb, -255);
  }
  //////////////////////////////////////
if (sprb > 0)
  {
    digitalWrite(DR1b, LOW);
    digitalWrite(DR2b, HIGH);
    analogWrite(PWMRb, sprb);
  }
else if (sprb < 0)
  {
    digitalWrite(DR1b, HIGH);
    digitalWrite(DR2b, LOW);
    analogWrite(PWMRb, -sprb);
  }
else
  {
    digitalWrite(DR1b, HIGH);
    digitalWrite(DR2b, HIGH);
    analogWrite(PWMRb, -255);
  }}

//////////////////////////////////////
void runc(int sprc, int splc)   // ประกาศฟังก์ชัน run(กำลังมอเตอร์ซ้าาย,กำลังมอเตอร์ขวา);
{ if (splc > 0)
  {
    digitalWrite(DL1c, LOW);
    digitalWrite(DL2c, HIGH);
    analogWrite(PWMLc, splc);
  }
else if (splc < 0)
  {
    digitalWrite(DL1c, HIGH);
    digitalWrite(DL2c, LOW);
    analogWrite(PWMLc, -splc);
    }
else
  {
    digitalWrite(DL1c, HIGH);
    digitalWrite(DL2c, HIGH);
    analogWrite(PWMLc, -255);
  }
  //////////////////////////////////////
if (sprc > 0)
  {
    digitalWrite(DR1c, LOW);
    digitalWrite(DR2c, HIGH);
    analogWrite(PWMRc, sprc);
  }
else if (sprc < 0)
  {
    digitalWrite(DR1c, HIGH);
    digitalWrite(DR2c, LOW);
    analogWrite(PWMRc, -sprc);
  }
else
  {
    digitalWrite(DR1c, HIGH);
    digitalWrite(DR2c, HIGH);
    analogWrite(PWMRc, -255);
  }}

void menu1()  /// code 1 ที่นี่
{
sv_knob(); //หาองศา servo moter
}
void menu2()   /// code 2 ที่นี่
{
Sensor_RGB();
}
void menu3()   /// code 3 ที่นี่
{ 
 P(Bypass);
}
void menu4()   /// code 4 ที่นี่
{ 
 L(Lsp,gyroL); // Turn Left
}
void menu5()   /// code 5 ที่นี่
{ 
 R(Rsp,gyroR); // Turn Right
}
void menu6()   /// code 6 ที่นี่
{ 
 B(spLB,spRB,Back); // Backward
}
void menu7()   /// code 7 ที่นี่
{ 
 PP(Bypass);
} 
void menu8()   /// code 8 ที่นี่
{
RUN4(255,-255); 
}
void menu9()   /// code 9 ที่นี่
{
 slideR(spSRB,spSLB,SLb);
}
void menu10()   /// code 10 ที่นี่
{
  slideR(spSRb,spSLb,SLb);
}
void menu11()   /// code 11 ที่นี่
{
  turnb(Tb,gyroT);
}
void menu12()   /// code 12 ที่นี่
{
  
}
