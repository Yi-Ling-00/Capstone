#include <Adafruit_BMP085.h>
#include <Wire.h>

Adafruit_BMP085 bmp;

//excel stuff
const byte kNumberOfChannelsFromExcel = 6; 
// Comma delimiter to separate consecutive data if using more than 1 sensor
const char kDelimiter = ',';    
// Interval between serial writes
const int kSerialInterval = 50;   
// Timestamp to track serial interval
unsigned long serialPreviousTime; 

char* arr[kNumberOfChannelsFromExcel];
// Connect VCC of the sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

/////////Variable definitions///////////
//Motor controller pins

#define LPWM_POS 10  //POS pump -PWM
#define RPWM_POS 11  //set LWPM for forward and SET RPWM for backward

#define LPWM_NEG 12  //NEG pump
#define RPWM_NEG 13  //PWM

//solenoid valves
#define svPos 37
#define svNeg 39

//Safety features define (LEDs don't need PWM...)
#define ledR 2
#define ledY 3
#define ledG 4
#define buzz 5


//Pressure sensor variables
double p_ref = 0.0;  //Initialize reference pressure [cmH2O]
double pressure;     //Initialize instantaneous pressure [cmH2O]
int N = 3;           //Number of data points to get reference pressure
int diff = 0;        //(Cuirass pressure w.r.t. p_ref) = pressure - p_ref

//Safety Variables
int posCap = 20;     //Default allowable +pressure [cmH2O]
int negCap = -20;    //Default allowable -pressure [cmH2O]
double warn = 1.10;  //% difference from input value when system notifies user that
int danger = 0;
double down = 1.20;  //% difference from input value when system shuts down

void setup() {
  Serial.begin(9600);

  pinMode(svNeg, OUTPUT);
  pinMode(svPos, OUTPUT);
  digitalWrite(svNeg, HIGH);  //close valve (default is on)
  digitalWrite(svPos, HIGH);  //close valve (default is on)

  pinMode(LPWM_POS, OUTPUT);
  pinMode(RPWM_POS, OUTPUT);
  pinMode(LPWM_NEG, OUTPUT);
  pinMode(RPWM_NEG, OUTPUT);
  analogWrite(LPWM_POS, 0);  //turn off pumps first (can remove this code later...)
  analogWrite(RPWM_POS, 0);  //turn off pumps first (can remove this code later...)
  analogWrite(LPWM_NEG, 0);  //turn off pumps first (can remove this code later...)
  analogWrite(RPWM_NEG, 0);  //turn off pumps first (can remove this code later...)

  pinMode(ledR, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(buzz, OUTPUT);

  // Check sensor connection
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  for (int i = 0; i < N; i++) {
    pressure = bmp.readPressure();  //measure pressure [Pascal]
    p_ref += pressure;              //add measured pressure
    delay(50);                      //Delay for next data point [msec]; Could be adjusted
  }
  p_ref /= N;          //take the average of N measurements as the reference pressure.
  p_ref *= 0.0101972;  //Convert to [cmH2O]

  Serial.print("****Pressure REF: ");
  Serial.print(p_ref);
  Serial.println(" cm H20");

  digitalWrite(ledR, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledG, HIGH);
}

void sendDataToSerial()
{
  Serial.print(diff);
  Serial.println(); // Add final line ending character only once
}

void processOutgoingSerial()
{
   // Enter into this only when serial interval has elapsed
  if((millis() - serialPreviousTime) > kSerialInterval) 
  {
    // Reset serial interval timestamp
    serialPreviousTime = millis(); 
    sendDataToSerial(); 
  }
}

void setNeg() {  //negative pressure pump ON at full power
  digitalWrite(svNeg, LOW);
  digitalWrite(svPos, HIGH);

  analogWrite(LPWM_POS, 0);
  analogWrite(RPWM_POS, 0);
  analogWrite(LPWM_NEG, 240);  //pwm value (forward is neg)
  analogWrite(RPWM_NEG, 0);
}

void setPos() {  //positive pressure pump ON at full power
  digitalWrite(svNeg, HIGH);
  digitalWrite(svPos, LOW);

  analogWrite(LPWM_POS, 0);
  analogWrite(RPWM_POS, 240);  //pwm value (reverse is pos)
  analogWrite(LPWM_NEG, 0);
  analogWrite(RPWM_NEG, 0);
}

void setOff(int t = 5000) {  //both pumps OFF
  analogWrite(LPWM_POS, 0);
  analogWrite(RPWM_POS, 0);
  analogWrite(LPWM_NEG, 0);
  analogWrite(RPWM_NEG, 0);

  digitalWrite(svNeg, HIGH);
  digitalWrite(svPos, HIGH);
}

void safety(int color){
  //turn off green
  digitalWrite(ledG, LOW);

  if (color == 1){ //red
    Serial.println("SAFETY RED");
    danger = 1;
    for (int i = 0; i < 10; i++) {
      digitalWrite(ledR, HIGH);
      analogWrite(buzz, 127);  
      delay(125);

      digitalWrite(ledR, LOW);
      analogWrite(buzz, 0);   
      delay(125); //time between beeps
    } 
  }
  else{//yellow
  Serial.println("WARNING YELLOW");
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledY, HIGH);
      analogWrite(buzz, 255);   
      delay(250);
      
      digitalWrite(ledY, LOW);
      analogWrite(buzz, 0);  
      delay(250); //time between beeps
    } 
  }

  digitalWrite(ledR, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledG, HIGH);
  analogWrite(buzz, 0);  
}

void sensorDiff() {  //Pressure difference from reference pressure
  pressure = 0.0101972 * bmp.readPressure();
  diff = pressure - p_ref;

  if (abs(diff) > 60){
    diff = 0; //sensor error reset
  }
  //Check Safety

  if ((diff < down * negCap) || (diff > down * posCap)) {//when threshold exceeds 20% of desired 
    //shuts off immediately?
    setOff();
    //RED LED + Buzzer
    safety(1);


  } else if ((diff < warn * negCap) || (diff > warn * posCap)) { //when threshold exceeds 10% of desired value 
    //Yellow LED + Buzzer
    safety(2);
  }

  //Display the pressure within cuirass w.r.t. the reference pressure
  // Serial.print("Pressure : ");
  // Serial.println(diff);
  processOutgoingSerial();
}

void NPV(int neg = -1) {
  String reply = "";
  //Update Safety Variables
  if (neg > 20){
    negCap = -neg;
  }

  while (reply != "exit" && danger == 0) {  // if reply == -1, exit
    sensorDiff();            //Retrieve data from pressure sensor
    // Serial.print("****Pressure REF: ");
    // Serial.println(p_ref);
    // Serial.print("------------------------PRESSURE: ");
    // Serial.println(pressure);

    while (diff > -neg) {
      setNeg();
      sensorDiff();
    }

    if (diff <= -neg) {  //turn off when pressure is reached (if the pressure drops, this SHOULD turn back on ???)
      // Serial.println("Desired pressure");
      setOff();
    }

    if (Serial.available() > 0) {
      reply = Serial.readString();
      reply.trim();
      // Serial.print("Buffer: ");
      // Serial.println(reply);
    }
  }

  setOff();
  // Serial.println("****************Exiting NPV mode****************");
  
  return;  //exit
}

void BPV(int breathe = 10, float in_IE = 2, float out_IE = 1, int pos = 5, int neg = 5) {
  float ttl_time, pos_time, neg_time;

  //print parameters
  Serial.print("* in_IE: ");
  Serial.println(in_IE);
  Serial.print("* out_IE: ");
  Serial.println(out_IE);
  Serial.print("* breathe: ");
  Serial.println(breathe);
  Serial.print("* pos: ");
  Serial.println(pos);
  Serial.print("* neg: ");
  Serial.println(neg);

  //Update Safety Variables
  if(neg > 20){
    negCap = -neg;  
  }
  if (pos > 20){
    posCap = pos;
  }
  //set parameters
  ttl_time = in_IE + out_IE;
  pos_time = (in_IE / ttl_time) * breathe;
  neg_time = (out_IE / ttl_time) * breathe;

  Serial.print("TOTAL TIME: ");
  Serial.println(ttl_time);  
  Serial.print("POS TIME: ");
  Serial.println(pos_time);
  Serial.print("NEG TIME: ");
  Serial.println(neg_time);

  //Looping between pos and neg
  double start_time, end_time, time_diff;  //to track time
  String reply = "";

  while (reply != "exit" && danger == 0) {
    start_time = millis();                                   //Save starting time
    while (diff < neg && time_diff < ((neg_time*1000))) { //converting time to milliseconds
      // Serial.println("~~~~~~~~~~~~~~~~~~IN NEGATIVE BPV~~~~~~~~~~~~~~~~~~");
      setNeg();
      sensorDiff();               //get pressure reading
      end_time = millis();           //Save current time
      time_diff = end_time - start_time;  //Calculate time past
      // Serial.print("Time Diff: ");
      // Serial.println(time_diff);
    }
    start_time = millis();  //Save starting time
    diff = 0;
    while (diff < pos && time_diff < ((pos_time*1000))) {  //continue pos press if both pressure isnt reached or half the time isn't reached
      // Serial.println("!!!!!!IN POSITIVE BPV!!!!!!!!!!!!!!!");
      setPos();
      sensorDiff();               //get pressure reading
      end_time = millis();           //Save current time
      time_diff = end_time - start_time;  //Calculate time past
      // Serial.print("Time Diff: ");
      // Serial.println(time_diff);
    }
    diff = 0;
    if (Serial.available() > 0) {
      reply = Serial.readString();
      reply.trim();
      // Serial.print("IN BUFFER---------------------- : ");
      // Serial.println(reply);
    }
  }

  setOff();
  // Serial.println("**************EXITING BPV");
  return; //exit
}

void loop() {
  danger = 0; //reset danger variable 
  //NPV(10);
  //BPV(10, 2, 1, 5, 5);
  if (Serial.available() > 0) {
    String buffer = Serial.readString();
    buffer.trim();
    Serial.println(buffer);

    if (buffer == "npv") {
      int neg = -1;
      Serial.println("NPV MODE");
      //*******************Parsing inputs*******************
      while (neg < 0 || neg > 30) {
        Serial.println("Enter negative pressure (in cmh20): ");
        while (Serial.available() == 0); //do nothing, wait until user input again

        if (Serial.available() > 0) {
          neg = Serial.parseInt();  //in [cmH2O]
          neg = abs(neg);
        }
        if (neg < 0 || neg > 30) {
          Serial.println("Please enter range between 0 and 60");
        }
      }

      NPV(neg);
    }

    else if (buffer == "bpv") {
      float in_IE = -1, out_IE = -1;
      int pos = -1, neg = -1, breathe = -1;
      Serial.println("BPV MODE");

      //*******************Parsing inputs*******************

      while (breathe < 0 || breathe > 15) {  //is 15 an ok range?
        Serial.println("Enter Respiratory rate: ");
        while (Serial.available() == 0);  //do nothing, wait until user input again
    
        if (Serial.available() > 0) {
          breathe = Serial.parseInt();  //breathing cycles (ex: 6 seconds/breathe)
          Serial.println(breathe);
        }
        if (breathe < 0 || breathe > 15) {
          Serial.println("*Invalid input");
          Serial.println("Please enter range between 0 and 15");
        }
      }

      while (in_IE < 0) {  //IE ratio range??
        
        Serial.println("Enter inhale IE ratio: ");
        while (Serial.available() == 0);  //do nothing, wait until user input again
          
        if (Serial.available() > 0) {
          in_IE = Serial.parseFloat();
          Serial.println(in_IE);
        }
        if (in_IE < 0 || in_IE > 15) {
          Serial.println("^Invalid input");
          Serial.println("Please enter range between 0 and 15");
        }
      }

      while (out_IE < 0) {  //IE ratio range??
        Serial.println("Enter exhale IE ratio: ");
        while (Serial.available() == 0)
          ;  //do nothing, wait until user input again
        if (Serial.available() > 0) {
          out_IE = Serial.parseFloat();
          Serial.println(out_IE);
        }
        if (out_IE < 0 || out_IE > 15) {
          Serial.println("^Invalid input");
          Serial.println("Please enter range between 0 and 15");
        }
      }

      while (pos < 0 || pos > 30) {
        Serial.println("Enter positive pressure: ");
        while (Serial.available() == 0)
          ;  //do nothing, wait until user input again
        if (Serial.available() > 0) {
          pos = Serial.parseInt();  //in [cmH2O]
          Serial.println(pos);
        }
        if (pos < 0 || pos > 30) {
          Serial.println("%Invalid input");
          Serial.println("Please enter range between 0 and 30");
        }
      }

      while (neg < 0 || neg > 30) {
        Serial.println("Enter negative pressure: ");
        while (Serial.available() == 0);  //do nothing, wait until user input again
        if (Serial.available() > 0) {
          neg = Serial.parseInt();  //in [cmH2O]
          neg = abs(neg);
        }
        if (neg < 0 || neg > 30) {
          Serial.println("&Invalid input");
          Serial.println("Please enter range between 0 and 30");
        }
      }

      BPV(breathe, in_IE, out_IE, pos, neg);
    }
    else {
      Serial.println("INVALID INPUT");
    }
  }
}
