// Controled by serial
// f = 250Hz 传动比3:1 大齿轮为34mm MS3=0 MS2=MS1=1 1/8 STEP
// 250*1.8/360/3*3.1415*34/8 = 5.56MM/S
  
#define SigPin 7 // port
#define StepAngle 1.8 // degree
#define TransmissionRatio 3 
#define Diameter 34 //mm
#define Division 16
#define Pi 3.1415926

float ExtrutionVelocity = 1;// mm/s
int interval;
struct stepper {
  bool power = true;
  bool dir;
  int interval = 2000;
};
stepper extrusion;

void setup() {
  Serial.begin(9600);
  pinMode(SigPin,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  float freq = (ExtrutionVelocity*Division*360*TransmissionRatio*Pi/(StepAngle*Diameter));
  extrusion.interval = (int)(500.0*1000.0/freq);
  Serial.println(extrusion.interval);
}

void loop() {
  if (Serial.available()>0) {
    String data = Serial.readString();
    Serial.println("Arduino receive string: "+data);
    if (data == "r")
    {
      extrusion.power = true;
      Serial.println("Extrusion start.");
    }
    else if (data == "s")
    {
      extrusion.power = false;
      Serial.println("Extrusion stop.");
    }
    else if  (data == "t")
    // this part is used for testing serial
    {
      if (digitalRead(LED_BUILTIN)){
        digitalWrite(LED_BUILTIN, LOW);
      }
      else {
        digitalWrite(LED_BUILTIN, HIGH);
        }        
    }
  }
  if (extrusion.power){
      digitalWrite(SigPin, HIGH);
      delayMicroseconds(extrusion.interval);
      digitalWrite(SigPin, LOW);
      delayMicroseconds(extrusion.interval);
  }
}
