#include <SoftwareSerial.h>

#include <Wire.h>

//Libraries
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <Adafruit_MPU6050.h>//https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>//https://github.com/adafruit/Adafruit_Sensor
#include <SoftwareSerial.h>

//BLE
SoftwareSerial HM10(2, 3); // RX = 2, TX = 3
char appData;  
String inData = "";

//motors
const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


void M1_advance(char Speed) ///<Motor1 Advance
{
 digitalWrite(M1,LOW);
 analogWrite(E1,Speed);
}
void M2_advance(char Speed) ///<Motor2 Advance
{
 digitalWrite(M2,HIGH);
 analogWrite(E2,Speed);
}
void M3_advance(char Speed) ///<Motor3 Advance
{
 digitalWrite(M3,LOW);
 analogWrite(E3,Speed);
}
void M4_advance(char Speed) ///<Motor4 Advance
{
 digitalWrite(M4,HIGH);
 analogWrite(E4,Speed);
}

void M1_back(char Speed) ///<Motor1 Back off
{
 digitalWrite(M1,HIGH);
 analogWrite(E1,Speed);
}
void M2_back(char Speed) ///<Motor2 Back off
{
 digitalWrite(M2,LOW);
 analogWrite(E2,Speed);
}
void M3_back(char Speed) ///<Motor3 Back off
{
 digitalWrite(M3,HIGH);
 analogWrite(E3,Speed);
}
void M4_back(char Speed) ///<Motor4 Back off
{
 digitalWrite(M4,LOW);
 analogWrite(E4,Speed);
}

void M1_stop() ///<Motor1 Stop
{
 digitalWrite(M1,LOW);
 analogWrite(E1,0);
}

void M2_stop() ///<Motor2 Stop
{
 digitalWrite(M2,LOW);
 analogWrite(E2,0);
}

void M3_stop() ///<Motor3 Stop
{
 digitalWrite(M3,LOW);
 analogWrite(E3,0);
}

void M4_stop() ///<Motor4 Stop
{
 digitalWrite(M4,LOW);
 analogWrite(E4,0);
}



//ACCELEROMETER
Adafruit_MPU6050 mpu;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float positionX = 0, positionY = 0, positionZ = 0;
unsigned long previousMillis = 0;

void readMPU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentMillis = millis();
    float dt = (currentMillis - previousMillis) / 1000.0;  // Time elapsed since last read in seconds
    previousMillis = currentMillis;

    // Integrate acceleration to get velocity
    velocityX += a.acceleration.x * dt;
    velocityY += a.acceleration.y * dt;
    velocityZ += a.acceleration.z * dt;

    // Integrate velocity to get position
    positionX += velocityX * dt;
    positionY += velocityY * dt;
    positionZ += velocityZ * dt;

    // Print the position
    Serial.print("Position X: ");
    Serial.print(positionX);
    Serial.print(", Y: ");
    Serial.print(positionY);
    Serial.print(", Z: ");
    Serial.println(positionZ);


    AccX = a.acceleration.x / 16384.0; // X-axis value
    AccY = a.acceleration.y / 16384.0; // Y-axis value
    AccZ = a.acceleration.z / 16384.0; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    GyroX = g.gyro.x /131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = g.gyro.y / 131.0;
    GyroZ = g.gyro.z / 131.0;
    // Correct the outputs with the calculated error values
    GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
    GyroY = GyroY - 2; // GyroErrorY ~(2)
    GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;
    // Complementary filter - combine accelerometer and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // Print the values on the serial monitor
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);
}

void setup() {
    //ACCELEROMETER
 	  Serial.begin(115200);
    while (!Serial)
      delay(10); // will pause Zero, Leonardo, etc until serial console opens
      
    Serial.println("Adafruit MPU6050 test!");
      
    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");
      
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }
      
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
    
    Serial.println("");
    delay(100);

    //BLE
    Serial.begin(9600);
    Serial.println("HM10 serial started at 9600");
    HM10.begin(9600); // set HM10 serial at 9600 baud rate

    //motors
    for(int i=3;i<9;i++)
        pinMode(i,OUTPUT);
    for(int i=11;i<13;i++)
        pinMode(i,OUTPUT);
}

//on receipt of data from BLE, move the motors
//multitask the motors/ble and the accelerometer readings so that they can run concurrently
//If the ble reads F move the motors forward, if the ble reads B move the motors backward
unsigned long previousMillisMotor = 0; 
unsigned long previousMillisAccel = 0; 
const long intervalMotor = 2000; 
const long intervalAccel = 500; 
unsigned long motorStartTime = 0;
const unsigned long motorRunTime = 2000;
                                                                                          
void loop() {
    unsigned long currentMillis = millis();

    //ACCELEROMETER
    if (currentMillis - previousMillisAccel >= intervalAccel) {
        previousMillisAccel = currentMillis;
        readMPU();
    }

    //BLE
    HM10.listen();  // listen the HM10 port
    while (HM10.available() > 0) {   // if HM10 sends something then read
      appData = HM10.read();
      inData = String(appData);  // save the data in string format
      Serial.write(appData);
    }

    if (Serial.available()) {           // Read user input if available.
      delay(10);
      char user_input = Serial.read();
      HM10.write(user_input);
      Serial.print("Written to Arduino's BLE: ");
      Serial.println(user_input);
      if ( user_input == 'F' || user_input == 'B') { // move the motors forward or backward
          if (currentMillis - previousMillisMotor >= intervalMotor) {
              previousMillisMotor = currentMillis;
              motorStartTime = currentMillis;  // Start the timer when the motors start running
              if (user_input == 'F') {
                  M1_advance(100);
                  M2_advance(100);
                  M3_advance(100);
                  M4_advance(100);
              } else if (user_input == 'B') {
                  M1_back(100);
                  M2_back(100);
                  M3_back(100);
                  M4_back(100);
              }
          }
      }
    }

    // Stop the motors after 2 seconds
    if (motorStartTime > 0 && currentMillis - motorStartTime >= motorRunTime) {
        M1_stop();
        M2_stop();
        M3_stop();
        M4_stop();
        motorStartTime = 0;  // Reset the timer
    }
}