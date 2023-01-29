// This is code for testing the X001T, KY 026 and old firebot flame sensors and the MH edge detector


// X001T
const int xDin = 7;
String xName = "X001T";

// KY 026
const int kDin = 6;
const int kAin = A5;
String kName = "KY 0026";

// Old firebot IR sensor
const int oDin = 5;
const int oAin = A4;
String oName = "Old Flame";

// MH edge detector
const int mDin = 4;
const int mAin = A3;
String mName = "MH";


const int samples = 100;
const int rate = 1000; // 1000 is one second


// returns the mean reading from number of samples
//    pin is the pin which the analog data being read form
//    samples is the number of samples to take per measurement
int readMean(int pin, int samples){
  int sum = 0;
  for(int i = 0; i < samples; i++){
    sum = sum + analogRead(pin);
  }
  sum = sum / samples;
  return sum;
}

// returns the median reading form number of samples
//    pin is the pin which the analog data being read form
//    samples is the number of samples to take per measurement
//    uses bubble sort to order samples
int readMedian(int pin, int samples){
  int raw[samples];
  for(int i = 0; i < samples; i++){
    raw[i] = analogRead(pin);
  }
  int temp = 0;
  for(int i = 0; i < samples; i ++){
    for(int j = i; j < samples - 1; j ++){
      if(raw[j] > raw[j + 1]){
        temp = raw[j];
        raw[j] = raw[j + 1];
        raw[j + 1] = temp;
      }
    }
  }
  return raw[samples / 2];
}

// returns the boolian value which occures most in a number of samples
//    pin is the pin which the digital data being read form
//    samples is the number of samples to take per measurement
int readMode(int pin, int samples){
  int countF = 0;
  int countT = 0;
  for(int i = 0; i < samples; i++){
    if(digitalRead(pin) == 0){
      countT += 1;
    }
    else{
      countF += 1;  
    } 
  }
  if(countF > countT){
    return 0;
  }
  else{
    return 1;
  }
}

// this displays the data for the various sensors
void display(int dPin, int aPin, String sName){
  if(aPin == 0){
    Serial.print(sName);
    Serial.print("  digitalMode: ");
    Serial.println(readMode(dPin, samples));
  }
  else{
    Serial.print(sName);
    Serial.print("  readMode: ");
    Serial.print(readMode(dPin, samples));
    Serial.print(", readMedian: ");
    Serial.println(readMedian(aPin, samples));
  }
  delay(rate);
}

void setup() {
  Serial.begin(9600);
  pinMode(mAin, INPUT);
  pinMode(mDin, INPUT);
}

void loop() {
  
  display(mDin, mAin, mName);
  //display(oAin, oDin, oName);
  //display(kAin, kDin, kName);
  //display(0, xDin, xName);
  delay(1000);
}
