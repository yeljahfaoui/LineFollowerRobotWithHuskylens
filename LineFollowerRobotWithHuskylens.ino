#include <HUSKYLENS.h>

#define ENA_PIN 3
#define IN1_PIN 2
#define IN2_PIN 4

#define ENB_PIN 5
#define IN3_PIN 7
#define IN4_PIN 8

#define MOTOR_SPEED 50

HUSKYLENS huskylens;
HUSKYLENSResult result;
int ID = 1;
float Kp = 1;
float Ki = 0;
float Kd = 0;
int speed = 0;
float error = 0;
float lastError = 0;
float proportional = 0;
float integral = 0;
float derivative = 0;
unsigned long time = 0;
unsigned long newTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  Wire.begin();
  while(!huskylens.begin(Wire)) {
    Serial.print("\n Check I2C");
    delay(100);
  }

  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(huskylens.request(ID) && huskylens.available()) {
    result = huskylens.read();
    error = map(result.xTarget, 0, 320, -160, 160);

    time = millis();
    unsigned long dt = time - newTime;
    newTime = time;

    proportional = Kp * error;
    integral = Ki * (integral + error) * dt;
    derivative = Kd * ((error - lastError) / dt);
    lastError = error;
    speed = proportional + integral + derivative;
    Serial.println(speed);
    setRobotSpeed(MOTOR_SPEED - speed, MOTOR_SPEED + speed);

  } else {
    setRobotSpeed(0, 0);
  }
}

void setRobotSpeed(int rightSpeed, int leftSpeed) {
  // Right motors
  if(rightSpeed > 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

  } else if(rightSpeed < 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  // Left motors
  if(leftSpeed > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);

  } else if(leftSpeed < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);

  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }

  analogWrite(ENA_PIN, abs(rightSpeed));
  analogWrite(ENB_PIN, abs(leftSpeed));
}
