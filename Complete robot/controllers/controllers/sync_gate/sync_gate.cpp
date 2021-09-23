#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <webots/PositionSensor.hpp>
#include <windows.h>
#include <webots/InertialUnit.hpp>
#include <dos.h>

// All the webots classes are defined in the "webots" namespace



using namespace webots;

#define Time_Step 16
#define Max_Speed 10



// create the Robot instance.
Robot *gate = new Robot();
Motor *l1 =gate->getMotor("l1");
Motor *l2 =gate->getMotor("l2");
Motor *r1 =gate->getMotor("r1");
Motor *r2 =gate->getMotor("r2");
//Motor *rightMotor =robot->getMotor("motor1");

int timeStep = (int)gate->getBasicTimeStep();








int main(int argc, char **argv) {

  

  
 l1->setPosition(INFINITY);
  //rightMotor->setPosition(INFINITY);

  


  

int counter =0;

  while (gate->step(timeStep) != -1) {
 
  if(counter==1){ // Ga
 
   l1->setPosition(1.5);
 l2->setPosition(0);
  r1->setPosition(1.5);
 r2->setPosition(0);
 }
 
else if(counter==188){

 l1->setPosition(1.5);
 l2->setPosition(1.5);
 r1->setPosition(1.5);
 r2->setPosition(1.5);
 }
 else if (counter==625){

 l1->setPosition(0);
 l2->setPosition(1.5); 
 r1->setPosition(0);
 r2->setPosition(1.5); 

  }
 else if (counter==813){
 
 l1->setPosition(0);
 l2->setPosition(0);
 r1->setPosition(0);
 r2->setPosition(0);
 
  } 
 else if(counter ==1250){
 counter=0;
 }   
counter ++;



}
  delete gate;
  return 0;
}