#define BLUETOOTH_BAUDRATE 9600
#define PWM_MAX_VALUE 65535
#define SENSOR_AMMOUNT 8
#define OPTIC_AMMOUNT 6
#define OPTIC_FIRST_INDEX 2
#define ENCODER_WINDOWS 20
#define WHEEL_RADIUS 0.032
#define P_GAIN 200
#define I_GAIN 1
#define D_GAIN 0

#define OFFSET_RIGHT 0.30
#define OFFSET_LEFT 0.30

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

    void setSpeed(float percentage) {
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
    int ganhoI = 0;
    
    
    PID (double, double, double, double, double);

    float calcOutputRight(int error) {
      return max(min(offsetR - (Kp * error + Ki * ganhoI), OFFSET_RIGHT),0);
      /*Ki and Kd not implemented :(*/
    }
    float calcOutputLeft(int error) {
      return max(min(offsetL + Kp * error + Ki * ganhoI, OFFSET_LEFT),0);
      /*Ki and Kd not implemented :(*/
    }
    void newAcc(int error){
      ganhoI += error*(millis()-lastTime);
      lastTime = millis();
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
PID pid(P_GAIN, I_GAIN, D_GAIN, OFFSET_RIGHT, OFFSET_LEFT);

Motor MotorRight(PA8, PB12, PB13, 0);
Motor MotorLeft (PA9, PB14, PB15, 0);

float velRight;
float velLeft;

void setup() {

  initPerif();

  MotorLeft.setSpeed(0.25);
  MotorRight.setSpeed(0.25);

  MotorLeft.foward();
  MotorRight.foward();
}

void loop() {
  int inputs[SENSOR_AMMOUNT];

  /*Log inputs*/
  for (int i = 0; i < SENSOR_AMMOUNT; i++) {
    inputs[i] = analogRead(analogInPin[i])> BLACK_WHITE_THRESHOLD;
    Serial.print("S ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(inputs[i]);
  }
  
  /*end of line?*/
  for (int i = 2; i < SENSOR_AMMOUNT; i++) {
    if (inputs[i] > 0)
      break;
    if (i==SENSOR_AMMOUNT-1)
      while(1){
          MotorLeft.setSpeed(0);
          MotorRight.setSpeed(0);
      }
  }
  

  /*Calc error*/
  int error = 0;
  for (int i = OPTIC_FIRST_INDEX; i < 5; i++){
    error += inputs[i] - inputs[i + (OPTIC_AMMOUNT / 2)];
  }
  
  pid.newAcc(error); 
  float outputRight = pid.calcOutputRight(error);
  float outputLeft = pid.calcOutputLeft(error);
  
  Serial.print("E: ");
  Serial.println(error);
  Serial.print("OR: ");
  Serial.println(outputRight);
  Serial.print("OL: ");
  Serial.println(outputLeft);

  /*apply PID*/
  
  MotorLeft.setSpeed(outputLeft*65535);
  MotorRight.setSpeed(outputRight*65535);
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
