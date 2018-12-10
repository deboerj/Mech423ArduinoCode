

/*MECH 423 Etch a Sketch Controller
 * December 3, 2018
 * Jen Sze 
 * Joshua De Boer
 * 
 */
 
// Library Includes
#include <Stepper.h>
#include <QueueArray.h>

// Define Constants
const int STEPS_PER_REV = 32;                               //number of steps per internal motor revolution
const int GEAR_RED = 64;                                    //amount of gear reduction
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;   //number of steps per geard output rotation 2048
const int HORIZ_STEPS = STEPS_PER_OUT_REV; 
const int VERT_STEPS = STEPS_PER_OUT_REV;
const int STEPPER_SPEED = 1000;
const int STEPPER_SPEED_NL = 1000;
const int BAUD_RATE = 9600;


// Define Global variablesQueueArray <int> imageData; //hold the image data packets
QueueArray <int> grayScaleValue;
QueueArray <int> imageData;
int currentX = 0;
int currentY = 0;
int dataState = 0;
int commandByte = 0;
int errorByte = 0;
int k = 0;
int tempStep = 0;

// initialize Low Res pix size
int nRes = 0;
int pX = 250;     //size of a pixel X direction in steps
int pY = 250;     //size of a pixel Y direction in steps
int bLX = 117;
int bLY = 117;
int nL = 0;

// Define instances of motor class
Stepper H_Step(STEPS_PER_REV, 2, 4, 3, 5);     // in1 = pin 4, in2 = pin 5, in3 = 6, in3 = 7
Stepper V_Step(STEPS_PER_REV, 8, 10, 9, 11);   // in1 = pin 8, in2 = pin 9, in3 = 10, in3 = 11

// FUNCTION DECLARATIONS/DEFINITIONS

// Draw a pixel -------------------------------------------------------------------------- 
void drawByte(int byteValue){
//  Serial.println("Draw Byte Function");
  
  if(byteValue == 0){
//    Serial.print("if");
    H_Step.step(nRes*pX+bLX);
    delay(500);
    H_Step.step(-bLX);
    V_Step.step(-(pY+bLY));
    delay(500);
    V_Step.step(bLY);
    currentY = currentY + pY;
    Serial.write(nRes);
    nRes =0;
  }
  else{
//    Serial.println("else");
//    Serial.write(nRes);
    nRes++;
    if( byteValue > 0 && byteValue < 51){
//      Serial.print("20");
      draw(10);
    }else if( byteValue > 50 && byteValue < 76){
//      Serial.print("10");      
      draw(9);    
    }else if( byteValue > 75 && byteValue < 101){
//      Serial.print("8");
      draw(7);
    }else if( byteValue > 100 && byteValue < 126){
//      Serial.print("6");
      draw(6);     
    }else if( byteValue > 125 && byteValue < 151){
//      Serial.print("5");
      draw(5);
    }else if( byteValue > 150 && byteValue < 176){
//      Serial.print("4");
      draw(4);
    }else if( byteValue > 175 && byteValue < 201){
//      Serial.print("3");
      draw(3);
    }else if( byteValue > 200 && byteValue < 226){
//      Serial.print("2");
      draw(2);
    }else if( byteValue > 225 && byteValue < 255){
//      Serial.print("1");
      H_Step.step(-pX);
    }else{
      errorByte = 1;  
    }
  }
}

// Motor Control
void draw(int denom){
//  Serial.println("Draw function");
  for(int k=0;k<denom;k++){
    
    if(k == 0 && denom != 1){
      H_Step.step(-(pX/denom+pX%denom));
      delay(100);
      V_Step.step(-(pY+bLY));       
      delay(100);
      V_Step.step(pY+bLY);
      delay(100);
    }else{
      H_Step.step(-(pX/denom));
      delay(100);
      V_Step.step(-(pY+bLY));       
      delay(100);
      V_Step.step(pY+bLY);
      delay(00);
    }
  }
  currentX = currentX + pX;
}
//Serial Port Read  -------------------------------------------------------------------
byte byteArray[3];
void readSerial(){
  int incomingByte = 0;
  
  // receive data and Store into queue:
  while (Serial.available() > 0) {
   // Store the serial data sent to 
    incomingByte = Serial.readBytesUntil('e',byteArray,3);
    //Serial.write(incomingByte);
    imageData.enqueue(byteArray[0]);
    //Serial.write(imageData.front()); 
    imageData.enqueue(byteArray[1]);
    //Serial.write(imageData.front()); 
    imageData.enqueue(byteArray[2]);
    //Serial.write(imageData.front());
  }

}

// Make grayscale queue 
void makeGray(){
  int res = 0;
  while(!imageData.isEmpty()){
//    Serial.write(55);
//    Serial.write(dataState);
    if(dataState == 0 && imageData.front() == 255){
//      Serial.write(imageData.front());
      imageData.dequeue();
      dataState++;
      
    }else if(dataState == 1){
      commandByte = imageData.dequeue();
      //Serial.write(commandByte);
      dataState = 0;
      
      if(commandByte == 0){
        //STOP BYTE 
        H_Step.step(0);
        V_Step.step(0);
        imageData.dequeue();          
              
      }else if(commandByte == 1){
        res = imageData.dequeue();
        
        if(res == 0){            
          Serial.write(100);
          // LOW RES set pX, pY 
          pX = 250; 
          pY = 250;
          
        }else if(res == 1){
          Serial.write(101);
          // MED RES set pX, pY 
          pX = 187; 
          pY = 187;
          
        }else if(res == 2){       
          Serial.write(102);
          // HIGH RES set pX, pY 
          pX = 125; 
          pY = 125;
          
        }else{
          errorByte = 1;
        }   
      }else if(commandByte == 2){
        //PAUSE PRINT
        H_Step.step(0);
        V_Step.step(0);
        imageData.dequeue(); //Discard byte 3 (don't need it)
        
      }else if(commandByte == 3){
        //RESUME PRINT
        commandByte = -1;
        imageData.dequeue(); //Discard byte 3 (don't need it)
        
      }else if(commandByte == 4){
        //ZERO MOTOR POSITION
        //V_Step.step(currentY);
        //H_Step.step(currentX);
        imageData.dequeue(); //Discard byte 3 (don't need it)
        
      }else if(commandByte == 5){
         //READ GRAYSCALE VALUE
//        Serial.write(105);                              // Debug statement
        grayScaleValue.enqueue(imageData.dequeue());
//        Serial.write(grayScaleValue.dequeue());         // Debug statement
      }else{
        errorByte = 1; // COULD NOT READ BYTE PACKAGE
      }//end else
    }else{
      imageData.dequeue();
      dataState = 0;
    }//end else
    
  }//end while
  
}//end makeGray

//Serial Port Write  -------------------------------------------------------------------
void writeSerial(int byte1, int byte2){
  Serial.write(255);
  Serial.write(byte1);
  Serial.write(byte2);
}

void setup() {
  //
  V_Step.setSpeed(STEPPER_SPEED);
  H_Step.setSpeed(STEPPER_SPEED);
  V_Step.step(pY);
  delay(500);
  V_Step.step(-bLY);
  delay(500);
  H_Step.step(pX);
  H_Step.step(-bLX);
  delay(500);
  
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  
  // receive image data from Serial port
  writeSerial(6,0);
  readSerial();

  // set resolution and enqueue gray scale values into their own queue  
    while(!imageData.isEmpty()){
//      Serial.write(44);
    makeGray();
    }  
  
    // draw the image  from the gray scale queue
    while(!grayScaleValue.isEmpty()){
//      Serial.write(grayScaleValue.dequeue());
      
      Serial.write(grayScaleValue.peek());
      drawByte(grayScaleValue.dequeue());

    }

}
