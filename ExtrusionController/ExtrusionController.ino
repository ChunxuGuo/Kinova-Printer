// Controled by serial

#define SigPin 7 // stepper signal
int freq = 500; //Hz
struct stepper {
  bool power = true;
  bool dir;
  int interval = 2000;
};
stepper extrusion;

void setup() {
  Serial.begin(9600);
  pinMode(SigPin,OUTPUT);
}

void loop() {
  if (Serial.available()>0) {
    String data = Serial.readString();
    Serial.println("Arduino receive string: "+data);
    if (data == "r\n")
    {
      extrusion.power = true;
      Serial.println("Extrusion start.");
    }
    else if (data == "s\n")
    {
      extrusion.power = false;
      Serial.println("Extrusion stop.");
    }
  }
  if (extrusion.power){
      digitalWrite(SigPin, HIGH);
      delayMicroseconds(extrusion.interval);
      digitalWrite(SigPin, LOW);
      delayMicroseconds(extrusion.interval);
  }
}
