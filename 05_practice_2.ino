#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT); 
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);  
  int i;
  for(i = 1; i <=10 ; i++)
  {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100); 
  }
  digitalWrite(PIN_LED, 1);
  while(1){;}
}

int toggle_state(int toggle){
  return toggle = 1 - toggle;
}
