#include <Arduino.h>

#include <L298N.h>
#include <hcsr04.h>
#include <Servo.h> 
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define D_SCLK 8 // Serial clock out (SCLK)
#define D_DIN 9 // Serial data out (DIN)
#define D_DC 10 // Data/Command select (D/C)
#define D_CS 11 // LCD chip select (CS)
#define D_RST 12 // LCD reset (RST)

// PIN definition
#define ENA 7
#define IN1 5
#define IN2 6
#define IN3 3
#define IN4 4
#define ENB 2

#define TRIG_PINA 46
#define ECHO_PINA 47
#define TRIG_PINC 44
#define ECHO_PINC 45
#define TRIG_PINB 42
#define ECHO_PINB 43

#define LEDA 41
#define LEDC 39
#define LEDB 37

#define ALARM 36

#define SERVO_DIST 13

#define IRL_IR 33
#define IRL_D1 A15
#define IRL_D2 A14
#define IRL_D3 A13
#define IRL_D4 A13
#define IRL_D5 A12
#define IRL_D6 A11
#define IRL_D7 A10
#define IRL_D8 A9

// Constants

const int D_CONTRAST = 35;

const int MODE_NORMAL = 0;
const int MODE_CRASH = 1;
const int MODE_LINE = 2;

const int US_MAX_DISTANCE = 200; // mm

const int SERVO_MIN_ANGLE = 60;
const int SERVO_MAX_ANGLE = 120;
const int SERVO_ERROR = -12;

const int DEFAULT_SPEED = 100;
const int MIN_SPEED = 75;
const int MAX_SPEED = 255;

const int TURN_NORMAL = 0;
const int TURN_SHARP = 1;
const int TURN_TWIST = 2;

const int IRL_MINVAL = 600;
const int IRL_MAXVAL = 950;
const int IRL_LEVEL = 3;
const int irlD[] = {IRL_D1, IRL_D2, IRL_D3, IRL_D4, IRL_D5, IRL_D6, IRL_D7, IRL_D8};


// Global objects
HCSR04 hcsr04A(TRIG_PINA, ECHO_PINA, 20, US_MAX_DISTANCE);
HCSR04 hcsr04B(TRIG_PINB, ECHO_PINB, 20, US_MAX_DISTANCE);
HCSR04 hcsr04C(TRIG_PINC, ECHO_PINC, 20, US_MAX_DISTANCE);
L298N motorA(ENA, IN1, IN2);
L298N motorB(ENB, IN3, IN4);
Servo servoDist;

// Software SPI (slower updates, more flexible pin options):
//Adafruit_PCD8544 display = Adafruit_PCD8544(D_SCLK, D_DIN, D_DC, D_CS, D_RST);
// Hardware SPI (faster, but must use certain hardware pins):
Adafruit_PCD8544 display = Adafruit_PCD8544(D_DC, D_CS, D_RST);

// GENERAL FUNCTIONS
void ledBlink3(int l) {
  digitalWrite(l, HIGH);
  delay(100);
  digitalWrite(l, LOW);
  delay(100);
  digitalWrite(l, HIGH);
  delay(100);
  digitalWrite(l, LOW);
  delay(100);
  digitalWrite(l, HIGH);
  delay(100);
  digitalWrite(l, LOW);
  delay(100);
}

//**************************************
//*** BEGIN CLASS PAPILLONCAR
//**************************************

class ArduCar 
{
  int autoMode;
  int currentMode;
  int currentSpeed;
  
  unsigned int updateInterval;
  unsigned int lastUpdate;

  public:
  ArduCar(long interval = 50)
  {
    autoMode = MODE_NORMAL;
    currentMode = 0;
    currentSpeed = DEFAULT_SPEED;
    lastUpdate = 0;
    updateInterval = interval;
    // Set crash Alarms
    pinMode(ALARM, OUTPUT);
    pinMode(LEDA, OUTPUT);
    pinMode(LEDC, OUTPUT);
    pinMode(LEDB, OUTPUT);
    // Set IR Line sensor
    pinMode(IRL_IR, OUTPUT);
  }

  void initMode()
  {
    // Ensure initial mode
    digitalWrite(ALARM, LOW);
    digitalWrite(LEDA, LOW);
    digitalWrite(LEDC, LOW);
    digitalWrite(LEDB, LOW);
    // Initial Display status
    display.begin();
    display.setContrast(D_CONTRAST);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.println("ArduCar!");
    display.setTextColor(WHITE, BLACK); // 'inverted' text
    display.println("V1.0");
    display.display();
    // Set servo
    servoDist.attach(SERVO_DIST);
    servoDist.write(0);
    delay(200);
    servoDist.write(120);
    delay(200);
    servoDist.write(60);
    delay(150);
    servoDist.detach(); // To avoid noise
    // Start in stop mode
    this->stop();
  }

  void start()
  {
    Serial.println("CAR START");
    if (autoMode != MODE_NORMAL)
      this->goForward(DEFAULT_SPEED);
  }

  void stop()
  {
    Serial.println("CAR STOP");
    motorA.stop();
    motorB.stop();
//    this->setSpeed(0);
  }

  void setForward()
  {
    this->resumeSpeed();
    motorA.forward();
    motorB.forward();    
  }

  void setBackward()
  {
    this->resumeSpeed();
    motorA.backward();
    motorB.backward();    
  }

  void setSpeed(int speed)
  {
    currentSpeed = speed;
    motorA.setSpeed(currentSpeed);
    motorB.setSpeed(currentSpeed);    
  }

  void resumeSpeed()
  {
    this->setSpeed(currentSpeed);
  }

  void goForward(int speed = DEFAULT_SPEED)
  {
    this->setSpeed(speed);
    this->setForward();
  }

  void goBackward(int speed = DEFAULT_SPEED)
  {
    this->setSpeed(speed);
    this->setBackward();
  }

  void turnRight(int turnType = TURN_NORMAL)
  {
    switch (turnType) {
      case TURN_NORMAL:
        motorA.setSpeed(currentSpeed / 2);
        motorB.setSpeed(currentSpeed);  
        motorA.forward();
        motorB.forward();    
        break;
      case TURN_SHARP:
        motorA.setSpeed(0);
        motorB.setSpeed(currentSpeed);    
        motorA.forward();
        motorB.forward();    
        break;
      case TURN_TWIST:
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(currentSpeed);
        motorA.backward();
        motorB.forward();    
        break;
    }
  }

  void turnLeft(int turnType = TURN_NORMAL)
  {
    switch (turnType) {
      case TURN_NORMAL:
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(currentSpeed / 2);  
        motorA.forward();
        motorB.forward();    
        break;
      case TURN_SHARP:
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(0);    
        motorA.forward();
        motorB.forward();    
        break;
      case TURN_TWIST:
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(currentSpeed);
        motorA.forward();
        motorB.backward();    
        break;
    }
  }

  void setAutoMode(int am)
  {
    autoMode = am;
    /*
    if (autoMode)
      this->setForward();
    else
      this->stop();
    */
  }

  bool readIRL(float* DD) {
    int irlV[8];
    float D = 0;
    int nD = 0;

    // IRL Start
    digitalWrite(IRL_IR, HIGH);
    delay(10);
    for (int i = 0; i < 8; i++) {
      irlV[i] = map(analogRead(irlD[i]), IRL_MAXVAL, IRL_MINVAL, 1, 5);
      if (irlV[i] < IRL_LEVEL) {
        D += i;
        nD++;
      }
    }
    // IRL Stop
    digitalWrite(IRL_IR, LOW);
    if (nD != 0) {
      D /= nD;
      D -= 3.5;
      *DD = D;
      return true;
    } else {
      return false;
    }
  }

  // Process Comnand
  void processCommand(char c) 
  {
    String command = "";
    int s;

    switch(c) {
      case 'F':
        command = "Adelante";
        this->setForward();
        break;
      case 'B':
        command = "Atrás";
        this->setBackward();
        break;
      case 'R':
        command = "Derecha";
        motorA.setSpeed(0);
        motorB.setSpeed(currentSpeed);    
        motorA.forward();
        motorB.forward();    
        break;
      case 'L':
        command = "Izquierda";
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(0);    
        motorA.forward();
        motorB.forward();    
        break;
      case 'I':
        command = "Adelante Derecha";
        motorA.setSpeed(currentSpeed / 2);
        motorB.setSpeed(currentSpeed);  
        motorA.forward();
        motorB.forward();    
        break;
      case 'J':
        command = "Atrás Derecha";
        motorA.setSpeed(currentSpeed / 2);
        motorB.setSpeed(currentSpeed);  
        motorA.backward();
        motorB.backward();    
        break;
      case 'G':
        command = "Adelante Izquierda";
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(currentSpeed / 2);  
        motorA.forward();
        motorB.forward();    
        break;
      case 'H':
        command = "Atrás Izquierda";
        motorA.setSpeed(currentSpeed);
        motorB.setSpeed(currentSpeed / 2);  
        motorA.backward();
        motorB.backward();    
        break;
      case 'S':
        command = "Stop";
        this->stop();
        break;
      case '0':
        command = "V=0";
        this->setSpeed(0);
        break;
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        command = "V=";
        command += c;
        s = map(c - '1', 1, 9, MIN_SPEED, MAX_SPEED);
        this->setSpeed(s);
        break;
      case 'q':
        command = "V=Max";
        this->setSpeed(MAX_SPEED);
        break;
      case 'X':
        command = "Auto Crash ON";
        this->setAutoMode(MODE_CRASH);
        break;
      case 'V':
        command = "Auto Line ON";
        this->setAutoMode(MODE_LINE);
        break;
      case 'x':
      case 'v':
        command = "Auto OFF";
        this->setAutoMode(MODE_NORMAL);
        this->stop();
        break;  
    }
    Serial.println(command);
  } 

  void update()
  {
    float D = 0;

    if((millis() - lastUpdate) > updateInterval) { 
      // time to update
      lastUpdate = millis();
      if (autoMode == MODE_CRASH) {
        int dA = hcsr04A.distanceInMillimeters();
        int dC = hcsr04C.distanceInMillimeters();
        int dB = hcsr04B.distanceInMillimeters();
        if ((dA > 0) || (dC > 0) || (dB > 0)) {
          digitalWrite(ALARM, HIGH);
          this->stop();
          if (dA > 0) {  // Obstacle at right
            motorA.setSpeed(currentSpeed);
            motorB.setSpeed(0);    
            motorA.forward();
            motorB.forward();    
            ledBlink3(LEDA);
          } else if (dC > 0) {  // Obstacle at center
            this->setBackward();
            delay(200);
            motorA.setSpeed(currentSpeed);
            motorB.setSpeed(0);    
            motorA.forward();
            motorB.forward();
            delay(200);
            ledBlink3(LEDC);
          }
          else if (dB > 0) { // Obstacle at left
            motorA.setSpeed(0);
            motorB.setSpeed(currentSpeed);    
            motorA.forward();
            motorB.forward();    
            ledBlink3(LEDB);
          }
          digitalWrite(ALARM, LOW);
        } else {
          // No crash alarm, restore forward direction
          this->setForward();
        }
      } else if (autoMode == MODE_LINE) {
        if (readIRL(&D)) {
          if (D < -2) {
            motorA.setSpeed(0);
            motorB.setSpeed(currentSpeed);  
            motorA.forward();
            motorB.forward();    
          } else if (D < -1) {
            motorA.setSpeed(currentSpeed / 3);
            motorB.setSpeed(currentSpeed);  
            motorA.forward();
            motorB.forward();
          } else if (D > 1) {
            motorA.setSpeed(currentSpeed);
            motorB.setSpeed(currentSpeed / 3);  
            motorA.forward();
            motorB.forward();    
          } else if (D > 2) {
            motorA.setSpeed(currentSpeed);
            motorB.setSpeed(0);  
            motorA.forward();
            motorB.forward();    
          } else {
            // About center, restore forward direction
            this->setForward();
          }
        } else {
          // No line read, restore forward direction
          this->setForward();
        }
      }
      // Read command
      char command = 0;
      if (Serial1.available()>0) { 
        command = Serial1.read();
      }
      if (command)
        this->processCommand(command);
    }
  }

  void testCar()
  {
    Serial.println("Forward Test Motor A and B");
    for (int i = MIN_SPEED; i < MAX_SPEED; i += 10) {
      Serial.print("Speed = ");
      Serial.println(i);
      this->setSpeed(i);
      this->setForward();
      delay(50);
    }
  
    this->stop();
    delay(1000);

    Serial.println("Backward Test Motor A and B");
    for (int i = MIN_SPEED; i < MAX_SPEED; i+=10) {
      Serial.print("Speed = ");
      Serial.println(i);
      this->setSpeed(i);
      this->setBackward();
      delay(50);
    }
    this->stop();
    delay (1000);

    Serial.println("Full Speed Forward");
    this->goForward(MAX_SPEED);
    delay(3000);
    this->stop();
    delay(1000);

    Serial.println("Full Speed Backward");
    this->goBackward(MAX_SPEED);
    delay (3000);

    Serial.println("Slow Speed Forward");
    this->goForward(MIN_SPEED);
    delay(3000);
    this->stop();
    delay(1000);

    this->currentSpeed = DEFAULT_SPEED;

    Serial.println("Left turns test");
    this->turnLeft(TURN_NORMAL);
    delay(2000);
    this->turnLeft(TURN_SHARP);
    delay(2000);
    this->turnLeft(TURN_TWIST);
    delay(2000);
    this->stop();
    delay(1000);

    this->currentSpeed = DEFAULT_SPEED;

    Serial.println("Right turns test");
    this->turnRight(TURN_NORMAL);
    delay(2000);
    this->turnRight(TURN_SHARP);
    delay(2000);
    this->turnRight(TURN_TWIST);
    delay(2000);

    this->stop();

  }

};
//**************************************
//*** END CLASS PAPILLONCAR
//**************************************

ArduCar arduCar;

void setup() 
{ 
  Serial.begin(115200);
  Serial1.begin(9600); 

  Serial.println("BEGIN");
  arduCar.initMode();
//  arduCar.testCar();
//  arduCar.setAutoMode(true);
  arduCar.start();
} 
 
 
void loop() 
{ 
  arduCar.update();
}
