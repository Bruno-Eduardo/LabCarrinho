#define CYCLIC_EXECUTIVE_PERIOD 100 /*millisec*/
#define ledPort PC13

void setup() {
  pinMode(ledPort, OUTPUT);
}
void loop() {
  digitalWrite(ledPort, HIGH);
  delay(CYCLIC_EXECUTIVE_PERIOD/2);
  digitalWrite(ledPort, LOW);
  delay(CYCLIC_EXECUTIVE_PERIOD/2);
}
