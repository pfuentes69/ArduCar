#include <Arduino.h>

#include <L298N.h>
#include <hcsr04.h>
#include <Servo.h> 

// PIN definition
#define ENA 7
#define IN1 5
#define IN2 6
#define IN3 3
#define IN4 4
#define ENB 2

#define ALARM 40

#define TRIG_PINA 46
#define ECHO_PINA 47
#define TRIG_PINC 44
#define ECHO_PINC 45
#define TRIG_PINB 42
#define ECHO_PINB 43

#define SERVO_DIST_PIN 8

// Constants

const int US_MAX_DISTANCE = 300; // mm

const int SERVO_MIN_ANGLE = 60;
const int SERVO_MAX_ANGLE = 120;
const int SERVO_ERROR = -12;

const int DEFAULT_SPEED = 150;
const int MIN_SPEED = 75;
const int MAX_SPEED = 255;

const int TURN_NORMAL = 0;
const int TURN_SHARP = 1;
const int TURN_TWIST = 2;

// Global objects
HCSR04 hcsr04A(TRIG_PINA, ECHO_PINA, 20, US_MAX_DISTANCE);
HCSR04 hcsr04B(TRIG_PINB, ECHO_PINB, 20, US_MAX_DISTANCE);
HCSR04 hcsr04C(TRIG_PINC, ECHO_PINC, 20, US_MAX_DISTANCE);
L298N motorA(ENA, IN1, IN2);
L298N motorB(ENB, IN3, IN4);

//**************************************
//*** BEGIN CLASS PAPILLONCAR
//**************************************

class ArduCar 
{
  bool autoMode;
  int currentMode;
  int currentSpeed;
  
  unsigned int updateInterval;
  unsigned int lastUpdate;

  public:
  ArduCar()
  {
    autoMode = false;
    currentMode = 0;
    currentSpeed = 0;
    lastUpdate = 0;
  }

  void configure(long interval) 
  {
    updateInterval = interval;
    // Set crash Alarm
    pinMode(ALARM, OUTPUT);
    digitalWrite(ALARM, HIGH);

    this->stop();
  }

  void start()
  {
    Serial.println("CAR START");
//    this->goForward(DEFAULT_SPEED);
  }

  void testCar()
  {
    Serial.println("Forward Test Motor A and B");
    for (int i = MIN_SPEED; i < MAX_SPEED; i += 10)
    {
      Serial.print("Speed = ");
      Serial.println(i);
      this->setSpeed(i);
      this->setForward();
      delay(50);
    }
  
    this->stop();
    delay(1000);

    Serial.println("Backward Test Motor A and B");
    for (int i = MIN_SPEED; i < MAX_SPEED; i+=10)
    {
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

  void stop()
  {
    Serial.println("CAR STOP");
    motorA.stop();
    motorB.stop();
//    this->setSpeed(0);
  }

  void setForward()
  {
    motorA.forward();
    motorB.forward();    
  }

  void setBackward()
  {
    motorA.backward();
    motorB.backward();    
  }

  void setSpeed(int speed)
  {
    currentSpeed = speed;
    motorA.setSpeed(currentSpeed);
    motorB.setSpeed(currentSpeed);    
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

  void turnLeft(int turnType = TURN_NORMAL)
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

  void turnRight(int turnType = TURN_NORMAL)
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

  // Set Mode
  void setMode(char m) {
    String newMode = "";
    int s;

    switch(m) {
      case 'F':
        newMode = "Adelante";
        this->setForward();
//        this->setSpeed(currentSpeed);
        break;
      case 'B':
        newMode = "Atrás";
        this->setBackward();
        break;
      case 'R':
        newMode = "Derecha";
        this->turnRight(TURN_SHARP);
        break;
      case 'L':
        newMode = "Izquierda";
        this->turnLeft(TURN_SHARP);
        break;
      case 'I':
        newMode = "Adelante Derecha";
        this->turnRight(TURN_NORMAL);
        break;
      case 'J':
        newMode = "Atrás Derecha";
        break;
      case 'G':
        newMode = "Adelante Izquierda";
        this->turnLeft(TURN_NORMAL);
        break;
      case 'H':
        newMode = "Atrás Izquierda";
        break;
      case 'S':
        newMode = "Stop";
        this->stop();
        break;
      case '0':
        newMode = "V=0";
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
        newMode = "V=";
        newMode += m;
        s = map(m - '1', 1, 9, MIN_SPEED, MAX_SPEED);
        this->setSpeed(s);
        break;
      case 'q':
        newMode = "V=Max";
        this->setSpeed(MAX_SPEED);
        break;
    }
    Serial.println(newMode);
  } 


  void update()
  {
    if((millis() - lastUpdate) > updateInterval) { 
      // time to update
      lastUpdate = millis();
      if (autoMode) {
        int dA = hcsr04A.distanceInMillimeters();
        int dB = hcsr04B.distanceInMillimeters();
        int dC = hcsr04C.distanceInMillimeters();

        if (dA > 0) 
          this->turnRight(TURN_SHARP);
        else if (dB > 0)
          this->turnLeft(TURN_SHARP);
        else if (dC > 0)
          this->goBackward();
  //      else
  //        this->goForward(currentSpeed);
      }
      // Read command
      char command = 0;
      if (Serial1.available()>0) { 
        command = Serial1.read();
      }
      if (command)
        setMode(command);
    }
  }

};
//**************************************
//*** END CLASS PAPILLONCAR
//**************************************

ArduCar arduCar;

void setup() 
{ 
  Serial.begin(115200);
  Serial1.begin(38400); 

  Serial.println("BEGIN");
  arduCar.configure(50);
//  arduCar.testCar();
  arduCar.start();
} 
 
 
void loop() 
{ 
  arduCar.update();
}
