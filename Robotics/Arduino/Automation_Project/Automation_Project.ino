#include <Encoder.h>
const int DIR1 = 5; // direction 1, DIR1=0,DIR=1, CW rotation.
const int DIR2 = 7; // direction 2, DIR1=0,DIR=1, CW rotation.
const int ENA = 6;  //PWM output pin connecting to the driver.
const int encoderPinA = 2;//pin for encoder's channel A.
const int encoderPinB = 3;//pin for encoder's channel B.
const double PPR = 640.0; // 80(gearRatio)*2(PPR)*4(SinceQuadmode)

volatile long lastPosition  = 0;
volatile long currentPosition = 0;//keeps track of the current position of the motor.
long lastTime = 0;
long lastPrintTime = 0;
int k = 0;

double desired_RPM = 40.0;
double current_RPM = 0.0;
double dt = 0.025;
double dutyCycleMap = 0.0;

//optimal p i d 4 3 0.8

//step
//50: 10 2 0.05
//30: 10 4 0.05
double pid_error = 0.0;
double pid_kp = 10.0;
double pid_ki = 3.0;
double pid_kd = .05;
double pid_integral = 0.0;
double pid_derivative = 0.0;
double pid_previous_error = 0.0;
double pid_output = 0.0;
double pid_setpoint = 0.0;
double pid_max_output  =  255.0;
double pid_min_output  =  -255.0;

double g = 100.0; //low pass constant
double y = 0.0; //low pass output

char state = 'M';


Encoder myEnc(encoderPinA, encoderPinB);//Initialize Encoder. (Quad Mode, Interrupt Enabled)

void setup() {

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENA, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor Run Test");
  //getState();
}

void loop() {
  if (state == 'A') {
    if (millis() - lastTime > dt * 1000) {
      computeRPM();
      compute_PID();
      motor_run();
      lastTime = millis();
    }
  }
  else
  {
    if (millis() - lastTime > dt * 1000) {
      compute_PID();
      motor_run();
      lastTime = millis();
    }
  }

  if (millis() - lastPrintTime > 100) {
    Serial.print(desired_RPM);
    Serial.print("\t");
    Serial.print(current_RPM);
    Serial.print("\t");
    Serial.println(y);
    lastPrintTime = millis();
  }
}


void motor_run() {
  if (dutyCycleMap > 0.5) {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    analogWrite(ENA, abs(dutyCycleMap));
    //   }
  }
  else if (dutyCycleMap < -0.5) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    analogWrite(ENA, abs(dutyCycleMap));
  }
  else {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    analogWrite(ENA, 0);
  }
}

void compute_PID() {
  //Calculate RPM
  currentPosition = myEnc.read();
  current_RPM = (double)(((currentPosition - lastPosition) / dt) / PPR * 60.0);

  //Apply Low Pass Filter
  y += g * (current_RPM - y) * dt / 60;

  //Compute PID OUTPUT
  pid_error = desired_RPM - y;
  pid_integral += pid_error * dt;
  pid_derivative = (pid_error - pid_previous_error) / dt;
  pid_output = (pid_kp * pid_error) + (pid_ki * pid_integral) + (pid_kd * pid_derivative);

  //Clamp PID OUTPUT to max and min values
  if (pid_output >= pid_max_output) {
    pid_output = pid_max_output;
  }
  if (pid_output <= pid_min_output) {
    pid_output = pid_min_output;
  }

  dutyCycleMap = pid_output;
  pid_previous_error = pid_error;
  lastPosition = currentPosition;
}

void computeRPM() {
  if (k * dt < 5) {
    desired_RPM = 0;
  }
  else if (k * dt < 15) {
    desired_RPM = 8 * k * dt - 40;
  }
  else if (k * dt < 25) {
    desired_RPM = 80;
  }
  else if (k * dt < 35) {
    desired_RPM = 280 - 8 * k * dt;
  }
  else {
    desired_RPM = 0;
  }
  k++;
}


void getState()
{
  Serial.println("Input M (manual) or A (automatic)");
  while (!Serial.available());

  if (Serial.available() > 0) {

    state = Serial.read();

    if (state == 'M')
    {
      Serial.println("Input Desired RPM");
      while (!Serial.available());
      if (Serial.available() > 0)
        desired_RPM = Serial.parseInt(); //string to int?
      Serial.print("RPM: "); Serial.println(desired_RPM);
    }

    else if (state == 'A')
    {
      Serial.println("Automatic Selected");


    }
    else {
      Serial.println("Invalid input");
    }
  }
}

