#include <SoftwareSerial.h>

#include <Wire.h>

//Libraries
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
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
#include <Adafruit_MPU6050.h>//https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>//https://github.com/adafruit/Adafruit_Sensor
Adafruit_MPU6050 mpu;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float positionX = 0, positionY = 0, positionZ = 0;
unsigned long previousMillis = 0;
// Kalman filter variables
float Q_angle = 0.001; // Process noise variance for the accelerometer
float Q_bias = 0.003; // Process noise variance for the gyro bias
float R_measure = 0.03; // Measurement noise variance

float angle = 0; // The angle calculated by the Kalman filter
float bias = 0; // The gyro bias calculated by the Kalman filter
float rate; // Unbiased rate calculated from the rate and the calculated bias

float P[2][2] = {{0,0},{0,0}}; // Error covariance matrix
// Variables to store offsets
float AccX_offset = 0, AccY_offset = 0, AccZ_offset = 0;
float GyroX_offset = 0, GyroY_offset = 0, GyroZ_offset = 0;

void calibrateSensors() {
    int numReadings = 1000; // Number of readings to take for calibration

    // Variables to accumulate readings
    float AccX_sum = 0, AccY_sum = 0, AccZ_sum = 0;
    float GyroX_sum = 0, GyroY_sum = 0, GyroZ_sum = 0;

    for (int i = 0; i < numReadings; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Accumulate readings
        AccX_sum += a.acceleration.x;
        AccY_sum += a.acceleration.y;
        AccZ_sum += a.acceleration.z;
        GyroX_sum += g.gyro.x;
        GyroY_sum += g.gyro.y;
        GyroZ_sum += g.gyro.z;

        delay(1); // Small delay to simulate real-world conditions
    }

    // Calculate average values to get offsets
    AccX_offset = AccX_sum / numReadings;
    AccY_offset = AccY_sum / numReadings;
    AccZ_offset = AccZ_sum / numReadings;
    GyroX_offset = GyroX_sum / numReadings;
    GyroY_offset = GyroY_sum / numReadings;
    GyroZ_offset = GyroZ_sum / numReadings;
}

void readMPU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Subtract offsets from readings
    a.acceleration.x -= AccX_offset;
    a.acceleration.y -= AccY_offset;
    a.acceleration.z -= AccZ_offset;
    g.gyro.x -= GyroX_offset;
    g.gyro.y -= GyroY_offset;
    g.gyro.z -= GyroZ_offset;

    unsigned long currentMillis = millis();
    float dt = (currentMillis - previousMillis) / 1000.0; // Convert to seconds
    previousMillis = currentMillis;

    // Calculate angle based on Accelerometer reading
    float accAngle = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;

    // Calculate the new angle
    rate = g.gyro.x;
    angle += dt * (rate - bias);

    // Update estimation error covariance
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    // Calculate angle and bias
    float y = accAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update estimation error covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    // Output
    Serial.print(" Angle = ");
    Serial.println(angle);
    Serial.print(" Position (cm) : x = ");
    Serial.print(a.acceleration.x);
    Serial.print(" y = ");
    Serial.print(a.acceleration.y);
    Serial.print(" z = ");
    Serial.println(a.acceleration.z);
}

void setup() {
  //motors
    for(int i=3;i<9;i++)
        pinMode(i,OUTPUT);
    for(int i=11;i<13;i++)
        pinMode(i,OUTPUT);

    M1_stop();
    M2_stop();
    M3_stop();
    M4_stop();

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
      
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
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
      
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
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
    calibrateSensors();

    //BLE
    Serial.begin(9600);
    Serial.println("HM10 serial started at 9600");
    HM10.begin(9600); // set HM10 serial at 9600 baud rate
}

//on receipt of data from BLE, move the motors
//multitask the motors/ble and the accelerometer readings so that they can run concurrently
//If the ble reads F move the motors forward, if the ble reads B move the motors backward
unsigned long previousMillisMotor = 0; 
unsigned long previousMillisAccel = 0; 
const long intervalMotor = 11000;
const long intervalAccel = 500; 
unsigned long motorStartTime = 0;
unsigned long motorRunTime = 11000;




void loop() {
    unsigned long currentMillis = millis();

    //ACCELEROMETER
    if (currentMillis - previousMillisAccel >= intervalAccel) {
        previousMillisAccel = currentMillis;
        readMPU();
    }

    // BLE
    HM10.listen(); // écoute du port HM10

    while (HM10.available() > 0) { // si HM10 envoie quelque chose alors lire
        M1_stop();
        M2_stop();
        M3_stop();
        M4_stop();

        appData = HM10.read(); // Lire les données provenant de HM10
        inData = String(appData); // Convertir les données en chaîne de caractères
        Serial.write(appData); // Afficher les données reçues sur le moniteur série
    }

    if (Serial.available()) { // Read user input if available
        delay(10);
        char user_input = Serial.read(); // Read user input character

        HM10.write(user_input); // Send user input to HM10
        Serial.print("Written to Arduino: ");
        Serial.println(user_input); // Display user input on serial monitor

        // Control motors based on user input
        if (currentMillis - previousMillisMotor >= intervalMotor) {
            previousMillisMotor = currentMillis;
            motorStartTime = currentMillis; // Start timer when motors start running

            // Control motors based on user input
            if (user_input == 'b') {
                // Advance all motors
                M1_advance(255);
                M2_advance(255);
                M3_advance(255);
                M4_advance(255);
                motorRunTime = 12000; // Set motor run time to 12 seconds
            } else if (user_input == 'f') {
                // Back all motors
                M1_back(255);
                M2_back(255);
                M3_back(255);
                M4_back(255);
                motorRunTime = 11000; // Set motor run time to 11 seconds
            }
        }
    }


    // Stop the motors after the specified run time
    if (motorStartTime > 0 && currentMillis - motorStartTime >= motorRunTime) {
        M1_stop();
        M2_stop();
        M3_stop();
        M4_stop();
        motorStartTime = 0;  // Reset the timer
    }
}