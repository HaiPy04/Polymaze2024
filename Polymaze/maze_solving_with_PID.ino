#include <QTRSensors.h>


// Define SoftwareSerial RX and TX pins
//SoftwareSerial mySerial(0, 1); // RX, TX (Replace 10, 11 with your actual pins if needed)




int aphase = 9;
int aenlb = 6;
int bphase = 5;
int benbl = 3;
int mode = 8;

int maxSpeed = 105;
int minSpeed = -75;
int P;
int I;
int D;

float Kp = 0.008;
float Ki = 0.0076;
float Kd = 0.1;

int rotationback = 70;
int rotation = 150;
int coeff = -1;
int minreflec = 100;
int maxreflec = 900;
int lastError = 0;

String path = "";
               //RSBRRRSLL
               //RLRRSLL
String lastValue;
int pos = 0;
bool IsDiscoveringTheMaze = true;
bool IsSolvingTheMaze = false;// false 

int button_calibration = 2;
int button_start = A3;
bool once = 1;
#define NUM_SENSORS 8  // number of sensors used
#define TIMEOUT 2500   // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 7  // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtr((unsigned char[]){ 10, 11, 12, A0, A1, A2, A4, A5 },
                 NUM_SENSORS, TIMEOUT, EMITTER_PIN);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool justright=false;
bool justleft=false;

const int delaytime= 150;


void setup() {
  pinMode(0, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenlb, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  pinMode(mode, OUTPUT);

  pinMode(button_calibration, INPUT);
  pinMode(button_start, INPUT);

  digitalWrite(mode, HIGH);

  while (digitalRead(button_calibration) == LOW) {}
  //10 seconds
  digitalWrite(13, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(13, LOW);
  forward_movement(0, 0);
  //mySerial.begin(9600);
  
  while (digitalRead(button_start) == LOW) {}
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  //mySerial.println("Beggining");


}


void loop() {
  if (IsDiscoveringTheMaze) Discover();
  else if (IsSolvingTheMaze) Solve();
  else {
    IsDiscoveringTheMaze = false;
    IsSolvingTheMaze = true;
    pos=0;
    //maxSpeed += 5;    
    //mySerial.println("finished the maze");
    delay(1000);
    for(int i=0;i<50;i++) path.replace("BB","B");
    //mySerial.println(path);
    delay(1000);
    digitalWrite(13, HIGH);
    //path[path.length()] = '0';

    for(int i=0;i<50;i++)ShortPath();
   // mySerial.println(path);
    while (digitalRead(button_start) == LOW) {}
    digitalWrite(13, LOW);
  }

}

void SolveWithHighSpeed(){
  unsigned int position = qtr.readLine(sensorValues);
  //if (pos>4)digitalWrite(0, HIGH);
  if ((sensorValues[1] > maxreflec) && (sensorValues[2] > maxreflec) && (sensorValues[3] > maxreflec) && (sensorValues[4] > maxreflec) && (sensorValues[5] > maxreflec) && (sensorValues[6] > maxreflec)) {
      forward_movement(0, 0);
      while((sensorValues[4] < minreflec)&& (sensorValues[3] < minreflec)){
        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(-80, -80);
      }
      unsigned int position = qtr.readLine(sensorValues);
      
      if(path[pos]=='R'){
        pos++;
        rotationback = -70;

        forward_movement(50, -150);
        delay(200);
        while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
          unsigned int position = qtr.readLine(sensorValues);
          forward_movement(75, -75);
        }
      }
      else if(path[pos]=='L'){
        pos++;
        rotationback = 70;
        forward_movement(-150, 50);
        delay(200);
        while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
          unsigned int position = qtr.readLine(sensorValues);
          forward_movement(-75, 75);
        }
      }
  }else if ((sensorValues[0] > maxreflec) && (sensorValues[1] > maxreflec) && (sensorValues[2] > maxreflec)) {
      pos++;
      rotationback = -70;
      forward_movement(0, 0);
      while((sensorValues[4] < minreflec)&& (sensorValues[3] < minreflec)){
        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(-80, -80);
      }
      forward_movement(50, -150);
      delay(200);
      while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
        unsigned int position = qtr.readLine(sensorValues);
        forward_movement(70, -70);
      }

  }else if ((sensorValues[5] > maxreflec) && (sensorValues[6] > maxreflec) && (sensorValues[7] > maxreflec)) {
    if (path[pos] == 'L') {
      pos++;
      rotationback = 70;
      forward_movement(0, 0);
      while((sensorValues[4] < minreflec)&& (sensorValues[3] < minreflec)){
        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(-80, -80);
      }
      forward_movement(-150, 50);
      delay(210);
      while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
        unsigned int position = qtr.readLine(sensorValues);
        forward_movement(-75, 75);
      }
    }else if (path[pos] == 'S') {
      pos++;
      forward_movement(150, 150);
      delay(100);
    }

  }else {
    rotationback = 75;
    PID_control();
    //if (pos > path.length()-1) pos = path.length()-1;
  }
}
void Solve() {
  unsigned int position = qtr.readLine(sensorValues);
  if ((sensorValues[0] > maxreflec) && (sensorValues[1] > maxreflec) && (sensorValues[2] > maxreflec) && (sensorValues[3] > maxreflec) && (sensorValues[4] > maxreflec) && (sensorValues[5] > maxreflec) && (sensorValues[6] > maxreflec) && (sensorValues[7] > maxreflec)) {
      forward_movement(70, 70);
      delay(70);
      unsigned int position = qtr.readLine(sensorValues);
      if((sensorValues[2] > maxreflec) && (sensorValues[3] > maxreflec) && (sensorValues[4] > maxreflec) && (sensorValues[5] > maxreflec)) 
      {
        forward_movement(0, 0);
        IsSolvingTheMaze = false;
    }else {
      if (path[pos] == 'S') {
      pos++;
      forward_movement(90, 90);
      delay(100);}

      else if(path[pos]=='R'){
        pos++;
        rotationback = -70;
        forward_movement(70, 70);
        delay(150);
        forward_movement(50, -150);
        delay(delaytime);
        while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
          unsigned int position = qtr.readLine(sensorValues);
          forward_movement(70, -70);
        }
      }
      else if(path[pos]=='L'){
        pos++;
        rotationback = 70;
        forward_movement(70, 70);
        delay(90);
        forward_movement(-150, 50);
        delay(delaytime);
        while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
          unsigned int position = qtr.readLine(sensorValues);
          forward_movement(-70, 70);
        }
      }
    }
  }else if ((sensorValues[0] > maxreflec) && (sensorValues[1] > maxreflec) && (sensorValues[2] > maxreflec)) {
    if (path[pos] == 'R') {
      pos++;
      rotationback = -70;
      forward_movement(70, 70);
      delay(150);
      forward_movement(50, -150);
      delay(delaytime);
      while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
        unsigned int position = qtr.readLine(sensorValues);
        forward_movement(70, -70);
      }
      }else if (path[pos] == 'S') {
      pos++;
      forward_movement(90, 90);
      delay(100);
    }

  }else if ((sensorValues[5] > maxreflec) && (sensorValues[6] > maxreflec) && (sensorValues[7] > maxreflec)) {
    if (path[pos] == 'L') {
      pos++;
      rotationback = 70;
      forward_movement(70, 70);
      delay(150);
      forward_movement(-150, 50);
      delay(delaytime);
      while ((sensorValues[3] < minreflec) && (sensorValues[4] < minreflec)) {
        unsigned int position = qtr.readLine(sensorValues);
        forward_movement(-70, 70);
      }
    }else if (path[pos] == 'S') {
      pos++;
      forward_movement(90, 90);
      delay(100);
    }

  }else {
    rotationback = 70;
    PID_control();
    //if (pos > path.length()-1) pos = path.length()-1;
  }
  //mySerial.println(path[pos]);
}

void Discover() {
  uint16_t positionLine = qtr.readLine(sensorValues);
  //Checking T line
  if ((sensorValues[1] > maxreflec) &&(sensorValues[2] > maxreflec) && (sensorValues[3] > maxreflec) && (sensorValues[4] > maxreflec) && (sensorValues[5] > maxreflec)&& (sensorValues[6] > maxreflec))  {
    rotationback = -70;
    forward_movement(70, 70);
    delay(70);
    uint16_t positionLine = qtr.readLine(sensorValues);

    if((sensorValues[2] > maxreflec) && (sensorValues[3] > maxreflec) && (sensorValues[4] > maxreflec) && (sensorValues[5] > maxreflec)) 
    {
      forward_movement(0, 0);
      IsDiscoveringTheMaze = false;
    }
    else {
      
      //lastValue = "I";
      justright=true;
      justleft=false;
      path += 'R';
      forward_movement(50, -150);
      delay(delaytime);

      while ((sensorValues[3] < minreflec)&&(sensorValues[4] < minreflec)) {

        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(70, -70);
      }

      //rotationback = -70;
      //PID_control();
    }

    }
  //}// Checking for dead end
   else if ((sensorValues[0] < minreflec) && (sensorValues[1] < minreflec) && (sensorValues[2] < minreflec) && (sensorValues[3] < minreflec) && (sensorValues[4] < minreflec) && (sensorValues[5] < minreflec) && (sensorValues[6] < minreflec) && (sensorValues[7] < minreflec)) {
      if ((once )&& (!justright)&& (!justleft)) {
        path += "B";
        forward_movement(0, 0);
        delay(50);
      }
      once = 0;
      
      forward_movement(-rotationback, rotationback);
  }// Checking Right Turns 
  else if ((sensorValues[0] > maxreflec) && (sensorValues[1] > maxreflec) && (sensorValues[2] > maxreflec)) {
    path += 'R';
    justright=true;
    justleft=false;
    rotationback = -70;

    forward_movement(70, 70);
    delay(90);
    forward_movement(50, -150);
    delay(delaytime);
    
    while ((sensorValues[3] < minreflec)&&(sensorValues[4] < minreflec)) {
        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(70, -70);
      }

    //PID_control();


  } // Checking left turn
  else if ((sensorValues[5] > maxreflec) && (sensorValues[6] > maxreflec) && (sensorValues[7] > maxreflec) && (sensorValues[0] < minreflec)&& (sensorValues[1] < minreflec) ) {
    forward_movement(70, 70);
    delay(90);
    rotationback = 70;
    uint16_t positionLine  = qtr.readLine(sensorValues);
    if ((sensorValues[0] < minreflec) && (sensorValues[1] < minreflec) && (sensorValues[2] < minreflec) && (sensorValues[3] < minreflec) && (sensorValues[4] < minreflec) && (sensorValues[5] < minreflec) && (sensorValues[6] < minreflec) && (sensorValues[7] < minreflec)) {
      path+= 'L';

      justleft=true;
      justright=false;
     
      forward_movement(-150, 50);
      delay(delaytime);

      
      while ((sensorValues[3] < minreflec)&&(sensorValues[4] < minreflec)) {
        uint16_t positionLine = qtr.readLine(sensorValues);
        forward_movement(-70, 70);
      }
     // PID_control();
    }
    else{
       path += 'S';
       justleft=true;
       justright=false;
       //PID_control();
       }
  } // PID
  else {
    rotationback = 70;
    once=1;
    justright=false;
    justleft=false;
    PID_control();

  }
  //mySerial.println(isTakingTintersection);
}
void PrintValues() {
  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    //mySerial.print(sensorValues[i]);
    //mySerial.print('\t');
  }
  //mySerial.println();
}
void PID_control() {
  uint16_t positionLine = qtr.readLine(sensorValues);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd;

  int motorSpeedA = 150 + motorSpeedChange;
  int motorSpeedB = 150 - motorSpeedChange;

  if (motorSpeedA > maxSpeed) {
    motorSpeedA = maxSpeed;
  }
  if (motorSpeedB > maxSpeed) {
    motorSpeedB = maxSpeed;
  }
  if (motorSpeedA < minSpeed) {
    motorSpeedA = minSpeed;
  }
  if (motorSpeedB < minSpeed) {
    motorSpeedB = minSpeed;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}
void ShortPath() {
  path.replace("BB","B");
  path.replace("RBR", "S");
  path.replace("RBS", "L");
  path.replace("LBR", "B");
  path.replace("SBS", "B");
  path.replace("SBR", "L");
  path.replace("RBL", "B");
}
void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    digitalWrite(aphase, LOW);
  } else {
    digitalWrite(aphase, HIGH);
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    digitalWrite(bphase, HIGH);
  } else {
    digitalWrite(bphase, LOW);
  }
  analogWrite(aenlb, speedA);
  analogWrite(benbl, speedB);
}
