#include <ESP32Encoder.h> 
#include <ESP32Servo.h>

#define motorPin 12
#define A 13
#define B 15
const int MAX_INTEGRAL = 1000;

ESP32Encoder encoder;
Servo servo;

struct PID{
    float kp; 
    float kd;
    float ki;
};
PID pid = {0.5f, 0.0025f, 0.001f};


unsigned long lastTime = 0;
float dt = 0.0;
float lastError;
float integral;

unsigned long prevTime;

int desiredPosition;
 
int threshold = 1000;
String strdesiredPosition = "0";
int lastPosition;

int getPID(int);
int setSpeed(int output);
int getPosition();

void setup() {
  pinMode(motorPin, OUTPUT);
  encoder.attachHalfQuad ( A, B );
  encoder.setCount (0);

  Serial.begin ( 9600 );

  lastTime = micros();

  servo.attach(motorPin, 1100, 1900);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
    if (Serial.available() > 0) {
    strdesiredPosition = Serial.readStringUntil('\r\n');
    desiredPosition = strdesiredPosition.toInt();
    }

    int speed = getPID(desiredPosition);
    setSpeed(speed);
    Serial.print("Position: ");
    Serial.print(getPosition());
    Serial.print(" Target: ");
    Serial.print(desiredPosition);
    Serial.print(" Output: ");
    Serial.println(speed);

 /*
    Serial.printf("DATA,%d,%ld\n", getPosition(), micros());
    delay(5);
*/
}

int getPID(int targetPosition){ 
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;  // to seconds
    
    if (dt < 0.001f) {
        return 0;  // Skip update if called too frequently
    }

    int currentPosition = getPosition();
    int error = targetPosition - currentPosition;

    float derivative = (error - lastError) / dt;
    
    integral += error;
    if (integral > MAX_INTEGRAL || integral < -MAX_INTEGRAL) {
      integral = 0;
    }

    float output = pid.kp * error + pid.kd * derivative - pid.ki*integral;
    
    lastTime = now;
    lastError = error; 

    return (int) output;
}

int getPosition(){
  long counts = encoder.getCount() / 2;
  double dposition = counts/5.12;
  int position = (int) dposition;
  return position;
}


int setSpeed(int output){
  //
    if (output >= 0) {
        output = 1500+  0.6375*constrain(output, 0, 255);
        //x*255/(1900-1500) + 1500
    } else {
        output = 1500-(0.6375* constrain(-output, 0, 255));
        //1500-(x*255/(1900-1500))
    }
    servo.writeMicroseconds(output);
    return 0;
}

float getVelocity(){
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;  // to seconds
    
    if (dt < 0.001f) {
        return 0;  // Skip update if called too frequently
    }

    int currentPosition = getPosition();
    int diff = currentPosition - lastPosition;
    float derivative = (float) diff /dt;
    lastTime = now;
    lastPosition = currentPosition;
    return derivative;
}

void test_motors(){
    
}