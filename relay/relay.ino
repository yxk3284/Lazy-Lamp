int relay = 4;
int button = 2;

int buttonstate = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(relay, OUTPUT);
  pinMode(button, INPUT);
  digitalWrite(relay, HIGH);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonstate = digitalRead(button);
  if(buttonstate == HIGH){
      digitalWrite(relay, LOW);
  }
  else{
      digitalWrite(relay, HIGH);

  }
}
