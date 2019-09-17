

int blinkDelay = 500;

void faster(){
  blinkDelay = 100;
}

void slower(){
  blinkDelay = 2000;
}

void setup() {
  pinMode(PA1, INPUT_PULLDOWN);
  pinMode(PA9, OUTPUT);
  attachInterrupt(PA1,faster, RISING);
  attachInterrupt(PA1,slower, FALLING);
}
void loop() {
  digitalWrite(PA9, HIGH);
  delay(blinkDelay);
  digitalWrite(PA9, LOW);
  delay(blinkDelay);
}
