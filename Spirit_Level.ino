//Include Library 
#include "Wire.h" //I2C Library 
#include "I2Cdev.h"
#include "MPU6050.h" // MPU6050 Library
#include "math.h"  //Math Library to use MATHS Functions
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMono12pt7b.h>

MPU6050 mpu; //Creating Object for MPU6050

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int push_button1 = 3;
int push_button2 = 4;
int buzzer = 5;
int battery_input = A0;

int16_t gyroX, gyroY, gyroZ, accX, accY, accZ, mpuTemp; //Variable to collect the raw data from MPU6050 Sensor
float CgyroX, CgyroY, CgyroZ, CaccX, CaccY, CaccZ; //Calebrated DATA variables
float Xang, Yang; // Acceleration Angle Variable 
float FNXang, FOXang=0, FNYang, FOYang=0; //FILTER -->NEW-->OLD-->X or Y -->Acceleration ANGLE
float GXang=0, GYang=0; //Gyroscope Angle Variable
float ROLL, PITCH; //X-angle || Y-angle
float ROLL_REF=0,PITCH_REF=0;
float TEMP;
boolean set_gyro_angles = false, for_referance_data = false;
int receive_count; //
int Power;
int value1,value2;
int state=0,mode=0, ref_state=1;
int buzzer_value = 150;
float dt; // Delta T --> time gap between consecutive loops
unsigned long  loop_timer; // Time Variable

void setup() { 
  Serial.begin(9600);
  pinMode(A0,INPUT); 
  pinMode(push_button1,INPUT);
  pinMode(push_button2,INPUT);
  pinMode(buzzer,OUTPUT);
  
  mpu.initialize(); //Initializing the MPU6050
  OFFSET();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  LOGO_DISPLAY();
  loop_timer = micros() + 4000;  
}

void loop()
{
  RAW_DATA();
  ACC_ANGLE_CALCULATION();
  GYRO_ANGLE_CALCULATION();
  COMPLIMENTARY_FILTER();

  
  value1 = digitalRead(push_button1);
  value2 = digitalRead(push_button2);
  // print data on push button toggling
  
  while(value1 == 1 && state==0)
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value1=digitalRead(push_button1);
    analogWrite(buzzer,0);
    mode=1;
  }

  
  if(mode==1)
  {
    display.invertDisplay(false);
    display.setFont();
    state=1;
    ROLL_DISPLAY();
  } 
  while(value1==1 && state==1 )
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value1=digitalRead(push_button1);
    analogWrite(buzzer,0);
    mode=2;
  }

  
  if(mode==2)
  {
    state=2;
    PITCH_DISPLAY();
  }
  while(value1==1 && state==2 )
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value1=digitalRead(push_button1);
    analogWrite(buzzer,0);
    mode=3;
  }


  if(mode==3)
  {
    state = 3;
    BOTH_ROLL_AND_PITCH_DISPLAY();
  }
  while(value1==1 && state==3 )
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value1=digitalRead(push_button1);
    analogWrite(buzzer,0);
    mode=4;
  }

  
  if(mode == 4)
  {
    state =4;
    TEMP_DISPLAY();
  }
  while(value1==1 && state==4)
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value1=digitalRead(push_button1);
    analogWrite(buzzer,0);
    mode=1;
  }


  if(value2 == 1 && ref_state == 1)
  {
    ROLL_REF = ROLL;
    PITCH_REF = PITCH;
    ref_state = 0;
    while(value2 == 1)
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value2=digitalRead(push_button2);
    analogWrite(buzzer,0);
  }
  }
  
  if(value2 == 1 && ref_state == 0)
  {
    ROLL_REF = 0;
    PITCH_REF = 0;
    ref_state = 1;
    while(value2==1)
  {
    analogWrite(buzzer,buzzer_value);
    delay(20);
    value2=digitalRead(push_button2);
    analogWrite(buzzer,0);
  }
  }

  
  //set the frequancy of void loop at 250Hz
  while(loop_timer > micros());
  loop_timer += 4000;
  dt=0.004;
}

void OFFSET()
{
  mpu.setXAccelOffset(-1701);
  mpu.setYAccelOffset(920);
  mpu.setZAccelOffset(765);
  mpu.setXGyroOffset(79);
  mpu.setYGyroOffset(-20);
  mpu.setZGyroOffset(-67);
}

void RAW_DATA()
{
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
}

//Function to calculate the angle from the RAW Accelerometer Date
void ACC_ANGLE_CALCULATION()
{ //Angle Calculation --> Arc Tangent(X-gravity/Z-gravity)*(RADIAN TO DEGREE)
  Yang = -atan2(accX,accZ)*RAD_TO_DEG; 
  Xang = atan2(accY,accZ)*RAD_TO_DEG;
}

//Low pass filter to filter the noise comming from the disturbance
void LOW_PASS_FILTER()
{ // [NEW_FILTER_DATA = a*OLD_FILTER_DATA + b*ACTUAL DATA] and [a + b = 1]
  FNXang = (0.96*FOXang + 0.04*Xang); //ACCELEROMETER_ROLL
  FNYang = (0.96*FOYang + 0.04*Yang); //ACCELEROMETER_PITCH
}

//To update the all previous data with new data
void UPDATE()
{
  //Updating the old filter value with new filter value
  FOXang = FNXang; 
  FOYang = FNYang;
  
}

void GYRO_ANGLE_CALCULATION()
{
  // Converting the Raw Gyro Data in the range of +-250°/s
  CgyroX = gyroX/131.0; 
  CgyroY = gyroY/131.0;
  
  //Angle Calculation --> θ = θ + ω*Δt
  GXang = GXang + CgyroX*dt - 0.0039;
  GYang = GYang + CgyroY*dt + 0.00205;
}

void COMPLIMENTARY_FILTER()
{/* Gyro angle have drift but noise free and Acc angle have noise but no angle drift. Here we are making complimentary filter with GYRO AND ACC angle.
  Gyro angle only work for short period of time because Δt become zero if sensor get some stationary position and working as HIGH PASS FILTER. 
  While Acc angle will come out in long period or stationary system */
 
  if(set_gyro_angles){                                                 //If the IMU is already started
    GXang = 0.94*GXang + 0.06*Xang;
    GYang = 0.94*GYang + 0.06*Yang;
    ROLL = GXang - ROLL_REF;
    PITCH = GYang - PITCH_REF;
  }
  else{                                                                //At first start
    GXang = Xang;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    GYang = Yang;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
}

void CALEBRATED_DATA()
{
  //To Get the GYRO DATA in the range of +-250°/s
  CgyroX = gyroX/131.0;
  CgyroY = gyroY/131.0;
  CgyroZ = gyroZ/131.0;

  //To get the ACC. DATA in the range of +-2g
  CaccX = accX/16384.0;
  CaccY = accY/16384.0;
  CaccZ = accZ/16384.0;
}


void BOTH_ROLL_AND_PITCH_DISPLAY()
{
   display.clearDisplay();

  // display temperature
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("ROLL ");
  display.setTextSize(2);
  display.setCursor(0,10);
  display.print(ROLL);
  
  // display humidity
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("PITCH ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(PITCH);
  BATTERY_LEVEL();
  display.display(); 
}

void PITCH_DISPLAY()
{
  display.clearDisplay();
  display.setFont(&FreeMono12pt7b);
  display.setTextSize(1);
  display.setCursor(0,25); //(35,0)
  display.print("PITCH:");
  display.setFont(&FreeSansBold12pt7b);
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(PITCH);
  display.setFont();
  BATTERY_LEVEL();
  display.display(); 
}

void ROLL_DISPLAY()
{
  display.clearDisplay();
  display.setFont(&FreeMono12pt7b);
  display.setTextSize(1);
  display.setCursor(0,25); //(35,0)
  display.print("ROLL:");
  display.setFont(&FreeSansBold12pt7b);
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(ROLL);
  display.setFont();
  BATTERY_LEVEL();
  display.display(); 
}

void TEMP_DISPLAY()
{
  mpuTemp = mpu.getTemperature();
  TEMP = mpuTemp/340.+36.53;
  display.clearDisplay();
  display.setFont(&FreeMono12pt7b);
  display.setTextSize(1);
  display.setCursor(0,25); //(35,0)
  display.print("TEMP:");
  display.setFont(&FreeSansBold12pt7b);
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(TEMP);
  display.print(" C");
  display.setFont();
  BATTERY_LEVEL();
  display.display(); 
}

void BATTERY_LEVEL()
{
  analogRead(battery_input);
  Power = map(analogRead(battery_input), 570,860,0,100);
  display.setTextSize(1);
  display.setCursor(100,0);
  display.print(Power);
  display.print("%");
  //if(Power<20)
//  {
//    analogWrite(buzzer,buzzer_value);
//    delay(500);
//    analogWrite(buzzer,0);
//    delay(200);
//  }
}

void LOGO_DISPLAY()
{
  display.invertDisplay(true);
  display.setFont(&FreeMonoBoldOblique12pt7b);
  display.setTextSize(1);
  display.setCursor(0,25);
  display.print("Spirit");
  display.setCursor(50,50);
  display.print("Level");
  display.display();
}
