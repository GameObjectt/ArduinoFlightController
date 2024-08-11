//Arduino flight controller
//By gemo
//This uses degrees and not radians which explains the very low PID gains.
#include <Wire.h>
#include <Servo.h>
#include <PPMReader.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int motor1Pin = 5, motor2Pin = 6, motor3Pin = 9, motor4Pin = 10;
Servo s1, s2, s3, s4;


double p_x = .11, p_y = .11, p_z = 0.5;
double i_x = .03, i_y = .03, i_z = 0;
double d_x = -.13, d_y =-.12, d_z = 0;


double offsetX = -321.78, offsetY = 241.76, offsetZ = 123.16;
double sOffsetX = 7.56, sOffsetY = 99.18, sOffsetZ = 15986.71;

double angleX, angleY;

int rollSensitivity = 5, pitchSensitivity = 5, yawSensitivity = 20; // Sensitivity for each axis on the radio controller

PPMReader rc = PPMReader(7, 7);
void setup() {

  InitialiseMPU();

  InitialiseESCS();

  Serial.begin(115200);
  //Wait for each ESC to be initialised
  delay(10000);

}

int64_t lastMicros = 0;
int16_t lastRoll, lastPitch, lastYaw;

float pGX, pGY, pGZ;

float iGX, iGY, iGZ;

double channels[7];

// Serial debug bools
bool showAccX, showAccY, showAccZ, showGyroX, showGyroY, showGyroZ, showP, showI, showRC, showPWM;
bool kill = false;

int watchdogMillis = 0;
void loop() {
  
  // Get RC channels values
  for(int i = 0; i<7; i++){
    channels[i] = rc.latestValidChannelValue(i+1, 0);
  }

  double roll = -map(channels[3], 1000, 2000, -rollSensitivity, rollSensitivity);
  double pitch = map(channels[1], 1000, 2000, -pitchSensitivity, pitchSensitivity);
  double yaw = map(channels[0], 1000, 2000, -yawSensitivity, yawSensitivity);
  double throttle = map(channels[2], 1000, 2000, 0, 255);
  
  GetMPUValues();

  // Remove before flight
  Debug();

  if(kill == false && channels[4] > 1250){
    //Calculate the angles from the accelerometer
    angleY =  atan2(AcX, AcZ)/PI*180;
    angleX =  atan2(AcY, AcZ)/PI*180;

    // Get dt from the micros() function
    float deltaTime = (float)(micros() - lastMicros) / 1000000;

    // Gonna use this a bit later coupled with the accelerometer to get more precise angles
    /*pGX += GyX * deltaTime; 
    pGY += GyY * deltaTime;*/

    pGZ += GyZ * deltaTime + yaw * deltaTime;

    // subtract the RC values to get a higher/lower desired angle
    double gPGX = -angleX - pitch / p_x;
    double gPGY = angleY - roll / p_y;

    iGX += gPGX * deltaTime; // integral
    iGY += gPGY * deltaTime;
    iGZ += pGZ * deltaTime;
    lastMicros = micros();

    double dGX = GyX + pitch - lastPitch;
    double dGY = GyY + roll - lastRoll;
    double dGZ = GyZ + yaw - lastYaw;

    double motor1PWM = throttle + -dGY * d_y - dGX*d_x + dGZ*d_z
      - p_y * gPGY - p_x * gPGX - p_z * pGZ
      - i_y * iGY - i_x * iGX - i_z * iGZ;

    double motor2PWM = throttle + -dGY*d_y + dGX*d_x - dGZ*d_z
      - p_y * gPGY + p_x * gPGX + p_z * pGZ
      - i_y * iGY + i_x * iGX + i_z * iGZ;

    double motor3PWM = throttle + dGY*d_y - dGX*d_x + dGZ*d_z
    + p_y * gPGY - p_x * gPGX + p_z * pGZ
    + i_y * iGY - i_x * iGX + i_z * iGZ;

    double motor4PWM = throttle + dGY*d_y + dGX*d_x - dGZ*d_z // D
    + p_y * gPGY + p_x * gPGX - p_z * pGZ // P
    + i_y * iGY + i_x * iGX - i_z * iGZ; // I


    writePPM(s1, constrain(motor1PWM, 0, 255));
    writePPM(s2, constrain(motor2PWM, 0, 255));
    writePPM(s3, constrain(motor3PWM, 0, 255));
    writePPM(s4, constrain(motor4PWM, 0, 255));
  }else{
    writePPM(s1, -30);
    writePPM(s2, -30);
    writePPM(s3, -30);
    writePPM(s4, -30);
  }
  //Failsafe 1: kill switch on channel 7
  if(channels[6] > 1250){
    kill = true;
    throttle = -2000;
    Serial.println("kill");
  }
  //Failsafe 2: 0.5s watchdog timer for receiver disconnect
  if(channels[0] > 500){
      watchdogMillis = millis();
  }
  else if(millis() - watchdogMillis > 500){
    kill = true;
  }
}