#include "CytronMotorDriver.h"
#include <ESP32Servo.h>

Servo clawL;
Servo clawR;
Servo bin;
Servo camera;

int leftMotorPWM = 19;
int leftMotorDIR = 18;
int rightMotorPWM = 5;
int rightMotorDIR = 17;
int servoclawL = 4;
int servoclawR = 16;
int binservo = 13;
int posclawR = 0.00;
int posclawL = 180.00;
int camerapin = 15;
int LidPinOpen = 27 ;
int LidPinClose = 26 ;

 
int posClosingclawL = 0;
int cameraPos = 0;


// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 4;
const int resolution = 8;


void straight(){
  ledcWrite(leftMotorPWM, 20);
  digitalWrite(leftMotorDIR, LOW);

  ledcWrite(rightMotorPWM, 20);
  digitalWrite(rightMotorDIR, HIGH);
}

void reverse(){
  ledcWrite(leftMotorPWM, 100);
  digitalWrite(leftMotorDIR, HIGH);

  ledcWrite(rightMotorPWM, 100);
  digitalWrite(rightMotorDIR, HIGH);
}

void stop(){
  ledcWrite(leftMotorPWM, 0);
  ledcWrite(rightMotorPWM, 0);
}


void lidOpen(){
  analogWrite(LidPinOpen,255);
  delay(2000);
  analogWrite(LidPinOpen, 0);
}

void lidClose(){
  analogWrite(LidPinClose,255);
  delay(2000);
  analogWrite(LidPinClose, 0);
}

void ClawOpen(){
// Normal Opening Of Claws
  /*for(int i = posclawR , j = posclawL ; i<=180 && j >=90 ; i++ , j--){
    clawL.write(j);
    clawR.write(i);

    posclawR = i;
    posclawL = j;
  }*/
  // For Better Capturing Technique
  clawR.write(100);
  delay(300);
  clawL.write(100);
  delay(300);
  clawR.write(180);
  delay(300);
  clawL.write(0);
  delay(300);
}

void ClawClose(){
  for(int i = posclawR , j = posclawL ; i>=0 && j <=180 ; i-- , j++  )
  {
    clawL.write(j);
    clawR.write(i);

    posclawR = i;
    posclawL = j;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("r");

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  clawR.setPeriodHertz(50); 
  clawL.setPeriodHertz(50); 
  bin.setPeriodHertz(50);

  clawL.attach(servoclawL,500,2400);
  clawR.attach(servoclawR,500,2400);
  bin.attach(binservo,500,2400);
  clawL.write(180); // Set at Closed
  clawR.write(0); // Set at Closed
  bin.write(0); // Set at Biodegradable

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDIR, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDIR, OUTPUT);

  // configure LEDC PWM
  ledcAttachChannel(leftMotorPWM, freq, resolution, pwmChannel);
  ledcAttachChannel(rightMotorPWM, freq, resolution, pwmChannel);
}


// autonomous functions
void loop()
{
  if (Serial.available() > 0)
  {
        String str = Serial.readStringUntil('\n');  // Read data until newline
        str.trim();  // Remove extra spaces or newlines
        const char* command = str.c_str(); // convert String to char*
        int arr[2];

        // converting char* to int array
        char commandCopy[str.length() + 1];  // Create a writable character array
        strcpy(commandCopy, str.c_str());    // Copy the string into it
        char *p = strtok(commandCopy, ",");
        
        size_t index = 0;

        while (p != nullptr && index<2)
        {
          arr[index++] = atoi(p);
          p = strtok(NULL, ",");
        }


        if (arr[0] == 1)

          {
            // for when object is detected
            lidOpen();
            delay(3000);
            ClawOpen();
            delay(2000);
            straight();
            delay(arr[1]*1000);
            stop();
            ClawClose();
            delay(2000);
            lidClose();
            delay(2000);  
            Serial.println("r");
          }

        else if (arr[0] == 2)
          {
            // for when nothing detected
            Serial.println("r");
          }
  }
}
