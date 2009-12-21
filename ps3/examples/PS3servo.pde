#include <Servo.h>
#include <Spi.h>
#include <ps3_usb.h>
#include <MemoryFree.h>
void setup(void);
void loop(void);

Servo servoOne;  // create servo object for first Servo
Servo servoTwo;  // create servo object for second Servo
PS3_USB  PS3Game;   // create an object for the PS3 Game Controller

int PositionOne, PositionTwo; //storage for servo positions
char servomode; // mode for servo control 0 = joystick 1 = accelerometer

void setup() 
{ 
  servoOne.attach(2);  // attaches the first servo on pin 0 to the servo object 
  servoTwo.attach(3);  // attaches the first servo on pin 1 to the servo object 
  PS3Game.init();
  Serial.begin( 9600 );
  Serial.println("PS3 Controller");
  Serial.print("freeMemory() reports ");
  Serial.println( freeMemory() );
} 

void loop() 
{ 
  PS3Game.task();                   // perform the regular USB routines
  
  if ((PS3Game.statConnected()) && (PS3Game.statReportReceived())){ // report received ?
    
    if(PS3Game.buttonChanged()){   // right and left buttons change mode joystick/Accelerometer
    
      if(PS3Game.buttonPressed(buLeft)) {
       
        servomode = 0;
        PS3Game.LEDRumble(psLED1);
      }
      if(PS3Game.buttonPressed(buRight)) {
        
        servomode = 1;
        PS3Game.LEDRumble(psLED2);
      }
    }
    
    if(servomode){ // Accelerometer mode
    
      PositionOne = constrain(PS3Game.getMotion(AccelerometerX), 400, 600);    // constrain to +/- 1g
      PositionOne = map(PositionOne, 400, 600, 0, 179);    // scale it to use it with the servo 
      PositionTwo = constrain(PS3Game.getMotion(AccelerometerY), 400, 600);    // constrain to +/- 1g
      PositionTwo = map(PositionTwo, 400, 600, 0, 179);    // scale it to use it with the servo 
    }
    else{  // Joystick mode
      PositionOne = map(PS3Game.getJoystick(leftJoystickX), 0, 255, 0, 179);    // scale it to use it with the servo 
      PositionTwo = map(PS3Game.getJoystick(leftJoystickY), 0, 255, 0, 179);    // scale it to use it with the servo  
    }
  }
  
  
  
  servoOne.write(PositionOne);   //sets the first servo position according to the scaled value 
  servoTwo.write(PositionTwo);   // sets the first servo position according to the scaled value 
  delay(15);                           // waits for the servo low time 

 
} 

