#define BLUETOOTH_BAUDRATE 115200
#define PWM_MAX_VALUE 65535
#define SENSOR_AMMOUNT 8
#define OPTIC_AMMOUNT 6
#define OPTIC_FIRST_INDEX 2
#define ENCODER_WINDOWS 20
#define WHEEL_RADIUS 0.032
// #define P_GAIN 0.0215
#define P_GAIN 0.3
#define I_GAIN 0
#define D_GAIN 100

#define ERRO_GRANDE 3
#define ERRO_MEDIO 2
#define ERRO_PEQUENO 1

//#define OFFSET_RIGHT_MAX 0.392795
#define OFFSET_RIGHT_MAX 0.36
#define OFFSET_RIGHT_MIN 0
#define OFFSET_LEFT_MAX 0.34
#define OFFSET_LEFT_MIN 0

#define BLACK_WHITE_THRESHOLD 1000

const short analogInPin[] = {PA0, PA1, PA4, PA5, PA6, PA7, PB0, PB1};

class Motor {

    short PWMpin;
    short leftPin;
    short rightPin;
    int pwmValue;

  public:
    Motor (short, short, short, int);

    void foward(void) {
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);
    }

    void backward(void) {
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
    }

    void stop_(void) {
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
    }

    void setSpeed(double percentage) {
      pwmValue = int(percentage * PWM_MAX_VALUE);
      pwmWrite(PWMpin, pwmValue);
    }
};

Motor::Motor (short PWMpn, short leftPn, short rightPn, int pwm) {
  PWMpin    = PWMpn;
  leftPin   = leftPn;
  rightPin  = rightPn;
  pwmValue  = pwm;
}

class PID {
    double  Kp;
    double  Ki;
    double  Kd;
    double  offsetR;
    double  offsetL;

  public:
    unsigned long lastTime = 0;
    double ganhoI = 0.0;
    double ganhoD = 0.0;
    float lastError = 0.0;

    PID (double, double, double, double, double);

    float calcOutputRight(float error) {
      return max(min(offsetR + (Kp * error + Ki * ganhoI + Kd * ganhoD), OFFSET_RIGHT_MAX), OFFSET_RIGHT_MIN);
      /*Ki and Kd not implemented :(*/
    }
    float calcOutputLeft(float error) {
      return max(min(offsetL - (Kp * error + Ki * ganhoI + Kd * ganhoD), OFFSET_LEFT_MAX), OFFSET_LEFT_MIN);
      /*Ki and Kd not implemented :(*/
    }
    void newAcc(float error) {
      unsigned long menos = millis() - lastTime;
      //      Serial.print("Tempo:");
      //      Serial.println(menos);
      ganhoI += error * (menos);
      ganhoD = (error - lastError) / (millis() - lastTime);
      lastTime = millis();
      lastError = error;
    }
};

PID::PID (double Pgain, double Igain, double Dgain, double OffsetR, double OffsetL) {
  Kp = Pgain;
  Ki = Igain;
  Kd = Dgain;
  offsetR = OffsetR;
  offsetL = OffsetL;
}


/*VARIAVEIS GLOBAIS, DSCP IC*/
PID pid(P_GAIN, I_GAIN, D_GAIN, OFFSET_RIGHT_MAX, OFFSET_LEFT_MAX);

Motor MotorRight(PA8, PB12, PB13, 0);
Motor MotorLeft (PA9, PB14, PB15, 0);

float velRight;
float velLeft;

int inputs[SENSOR_AMMOUNT];
int sensor_i = 0;
short endOfLine = 0;
unsigned long tempo;
int lastError = 0;

void setup() {

  initPerif();

  MotorLeft.foward();
  MotorRight.foward();
}

void loop() {

  /*Log inputs*/
  for (sensor_i = 0; sensor_i < SENSOR_AMMOUNT; sensor_i++) {
    delay(2);
    inputs[sensor_i] = analogRead(analogInPin[sensor_i]) > BLACK_WHITE_THRESHOLD;
    //    Serial.print("S ");
    //    Serial.print(sensor_i);
    //    Serial.print(": ");
    //    Serial.println(analogRead(analogInPin[sensor_i]));
  }

  /*end of line?*/

  for (int i = 2; i < SENSOR_AMMOUNT; i++) {
    if (inputs[i] > 0) {
      endOfLine = 0;
      break;
    }
    else if (i == SENSOR_AMMOUNT - 1) {
      if (endOfLine == 0) {
        endOfLine = 1;
        tempo = millis();
      }
      else if (endOfLine == 1 && ((millis() - tempo) > (unsigned long)100)) {
        while (1) {
          MotorLeft.setSpeed(0);
          MotorRight.setSpeed(0);
        }
      }
    }
  }




  /*Calc error*/
  int error = 0;
  int j, i;

  if (not(inputs[2]) && not(inputs[3]))
    error = -(ERRO_GRANDE);
  else if (not(inputs[3]) && not(inputs[4]))
    error = -(ERRO_MEDIO);
  else if (not(inputs[4]) && not(inputs[5]))
    error = 0;
  else if (not(inputs[5]) && not(inputs[6]))
    error = (ERRO_MEDIO);
  else if (not(inputs[6]) && not(inputs[7]))
    error = (ERRO_GRANDE);
  else if (not(inputs[4]))
    error = -(ERRO_PEQUENO);
  else if (not(inputs[5]))
    error = (ERRO_PEQUENO);
  else if (not(inputs[3]))
    error = -(ERRO_MEDIO);
  else if (not(inputs[6]))
    error = (ERRO_MEDIO);
  else if (not(inputs[2]))
    error = -(ERRO_GRANDE);
  else if (not(inputs[7]))
    error = (ERRO_GRANDE);

  if (endOfLine)
    error = lastError;

  pid.newAcc(error);
  double outputRight = pid.calcOutputRight(error);
  double outputLeft = pid.calcOutputLeft(error);

  Serial.print("E: ");
  Serial.println(error);
  Serial.print("OR: ");
  Serial.println(outputRight * PWM_MAX_VALUE);
  Serial.print("OL: ");
  Serial.println(outputLeft * PWM_MAX_VALUE);

  /*apply PID*/

  MotorLeft.setSpeed(outputLeft);
  MotorRight.setSpeed(outputRight);

  lastError = error;
}

void initPerif() {
  pinMode(PA8, PWM);
  pinMode(PA9, PWM);

  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);

  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);

  //  attachInterrupt(PA0,encoderLeftHandler,RISING);
  //  attachInterrupt(PA1,encoderRightHandler,RISING);

  Serial.begin(BLUETOOTH_BAUDRATE);
}
//
//void encoderLeftHandler(){
//  static unsigned long lastTimeWasCalled = millis();
//
//  velLeft = (0.314159)*WHEEL_RADIUS/int(millis()-lastTimeWasCalled);
//  lastTimeWasCalled = millis();
//}
//
//void encoderRightHandler(){
//  static unsigned long lastTimeWasCalled = millis();
//
//  velRight = (0.314159)*WHEEL_RADIUS/int(millis()-lastTimeWasCalled);
//  lastTimeWasCalled = millis();
//}


//EOF
