#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/InertialUnit.hpp>
#include <cmath>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define TIME_STEP 16

double left2_ir_val;
double left1_ir_val;
double leftcenter_ir_val;
double rightcenter_ir_val;
double right1_ir_val;
double right2_ir_val;
double left_ps_val;
double right_ps_val;
int threshold=300;
int base_speed=5;
double turn_speed=6.28;

double target =750;
double last_wr_error = 0;
double last_wl_error = 0;
double final_error = 0;

const double position_base1 =0.88;
const double position_arm11 = -1.1;
const double position_arm22 =-0.5;
   
const double position_base =-2.12;
const double position_arm1 = -0.14;
const double position_arm2 =2.18;
int color_difference=0;

double get_error(){
    return  (-3*left2_ir_val-2*left1_ir_val- leftcenter_ir_val+rightcenter_ir_val+2*right1_ir_val+3*right2_ir_val)/350;
}

void ir_update(DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){
      left2_ir_val= irl2->getValue();
      left1_ir_val= irl1->getValue();
      leftcenter_ir_val= ircl->getValue();
      rightcenter_ir_val= ircr->getValue();
      right1_ir_val= irr1->getValue();
      right2_ir_val= irr2->getValue();
   
      if(left2_ir_val>threshold){
          left2_ir_val = 0;
        }
        else{
          left2_ir_val = 1;
        }
        if(left1_ir_val>threshold){
          left1_ir_val = 0;
        }
        else{
          left1_ir_val = 1;
        }
        if(leftcenter_ir_val>threshold){
          leftcenter_ir_val = 0;
        }
        else{
          leftcenter_ir_val = 1;
        }
        if(rightcenter_ir_val>threshold){
           rightcenter_ir_val = 0;
        }
        else{
           rightcenter_ir_val = 1;
        }
        if(right1_ir_val>threshold){
           right1_ir_val= 0;
        }
        else{
           right1_ir_val= 1;
        }
        if(right2_ir_val>threshold){
           right2_ir_val= 0;
        }
        else{
           right2_ir_val = 1;
        }
 
}


// 90 degree left turn
void turn_left(float thr,Robot *robot,Motor *left_motor,Motor *right_motor,PositionSensor *left_ps,PositionSensor *right_ps,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){
         float prev_val=(left_ps->getValue());
         bool over=true;
         float curr_val;
         left_motor->setVelocity(-0.1*turn_speed);
         right_motor->setVelocity(turn_speed);
         while ((robot->step(TIME_STEP) != -1) && over){
           
            curr_val=(left_ps->getValue());
            //std::cout<<"Error "<<(-1*(curr_val-prev_val))<<std::endl;
            if(-1*(curr_val-prev_val)>thr){
              over=false;
              ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
            
            
            
            }
   
         }
          
 }

// 90 degree right turn
void turn_right(float thr,Robot *robot,Motor *left_motor,Motor *right_motor,PositionSensor *left_ps,PositionSensor *right_ps,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){
         bool over=true;
         float prev_val=(right_ps->getValue());
         float curr_val;
         left_motor->setVelocity(turn_speed);
         right_motor->setVelocity(-0.1*turn_speed);
         while ((robot->step(TIME_STEP) != -1) && over){
           
            curr_val=(right_ps->getValue());
            if(-1*(curr_val-prev_val)>thr){
              over=false;
              ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
              }
         }
 }
 
void about_turn(Robot *robot,Motor *left_motor,Motor *right_motor,PositionSensor *left_ps,PositionSensor *right_ps,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,float thr){
         float prev_val=(left_ps->getValue());
         bool over=true;
         float curr_val;
         left_motor->setVelocity(-turn_speed);
         right_motor->setVelocity(turn_speed);
         while ((robot->step(TIME_STEP) != -1) && over){
           
            curr_val=(left_ps->getValue());
            //std::cout<<"Error "<<(-1*(curr_val-prev_val))<<std::endl;
            if(-1*(curr_val-prev_val)>thr){
              over=false;
              ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
            
            
            
            }
   
         }
          
 }
 
void move_straight(Robot *robot,Motor *left_motor,Motor *right_motor,int delay){
  int co =0;
  while((robot->step(TIME_STEP) != -1) && co<=delay){
  left_motor->setVelocity(base_speed);
  right_motor->setVelocity(base_speed);
  co++;
}
}

void stop(Motor *left_motor,Motor *right_motor){
   left_motor->setVelocity(0);
   right_motor->setVelocity(0);
      
}



double move_single(Robot *robot,Motor *left_motor,Motor *right_motor, DistanceSensor *ds_right_maze ){
  int co =0;
  int min=1000;
  while((robot->step(TIME_STEP) != -1) && co<=160){
  if(co==0){
    left_motor->setVelocity(0.1*base_speed);
    right_motor->setVelocity(-0.1*base_speed);
  }
  if(co==80){
    left_motor->setVelocity(-0.1*base_speed);
    right_motor->setVelocity(0.1*base_speed);
  }
  if(co==160){
    left_motor->setVelocity(0);
    right_motor->setVelocity(0);
  }
  double dis = ds_right_maze->getValue();
  if (dis<min){
    min=dis;
  }
  //std::cout << "ds_right: " << dis <<std::endl;
  co++;
  }
  
  return min;
 }
  


void single_linefollow(Motor *left_motor,Motor *right_motor,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){
      
      ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
      double P= get_error();
      double D = P - final_error;

      final_error = P;
      //std::cout<<"Error "<<P<<std::endl;

      left_motor->setVelocity(base_speed+200*P+500*D);
      right_motor->setVelocity(base_speed-200*P-500*D);

}

int color_detection(Camera *cam){
 
    const int width = cam->getWidth();
    const int height = cam->getHeight();
    const unsigned char *s_im  = cam->getImage();
     
    int r = 0, g = 0, b = 0;
    for (int x = width/4; x < 3*width/4; x++)
       for (int y = height/3; y < 4*height/5; y++) {
             r += cam->imageGetRed(s_im, width, x, y);
             g += cam->imageGetGreen(s_im, width, x, y);
             b += cam->imageGetBlue(s_im, width, x, y);
             //std::cout<< "r "<<r << " g " <<g << " b "<<b << std::endl;
      }
      
    int color_array[3] = {r,g,b};
    int ind = 0;
    for(int i = 0; i <3; i++){
         if(color_array[i] > color_array[ind]){
             ind = i;
         }
     }
     if (ind == 0){
         std::cout << "Detected color is RED" << std::endl;
         return 1;
     }else if (ind == 1){
         std::cout << "Detected color is GREEN" << std::endl;
         return 2;
     }else{
         std::cout << "Detected color is BLUE" << std::endl;
         return 3;
     }
}

void arm_function(Camera *cam,Robot *robot,Motor*r_motor,Motor*l_motor,Motor*base_motor,Motor*arm1_motor,Motor*arm2_motor){
    //camera on
    // detect front side
    bool arm_finish=true;
    int front_color=0;
    int bottom_color=0; 
    cam->enable(TIME_STEP);
    int counter=0;
    while ((robot->step(TIME_STEP) != -1) && arm_finish){
   
   if (counter==1){
      counter = counter;
      front_color = color_detection(cam);
      cam->disable();
      
   }else if(counter==3){
      arm2_motor->setPosition(position_arm2);
      counter = counter;

   } 
   else if (counter==80){
    base_motor->setPosition(position_base);
   counter = counter;
       
   }
   else if (counter==160){
   arm1_motor->setPosition(position_arm1);
   counter = counter;
       
   }
   
   
   else if (counter==600){
    r_motor->setPosition(0.1);
    l_motor->setPosition(-0.1);
   counter = counter;
   }
    
    
    else if (counter==800){
    base_motor->setPosition(position_base+1.2);
   counter = counter;
   }
 
    else if (counter==920){
    arm2_motor->setPosition(position_arm2-2.4);
    }
    
    // detect bootom face when counter 300
    else if (counter==1250){
    cam->enable(TIME_STEP);
    }
    
    else if (counter == 1251){
    bottom_color = color_detection(cam);
    cam->disable();
    }
   
    
    else if (counter==1500){
    arm2_motor->setPosition(position_arm2);
    }
    
    
    else if (counter==1700){
     base_motor->setPosition(position_base);
    }
    
    else if (counter==1800){
     r_motor->setPosition(-0.3);
     l_motor->setPosition(0.3);
     
    }
    
    else if (counter==2000){
    base_motor->setPosition(position_base1);
    }
    
    else if (counter==2200){
      arm2_motor->setPosition(position_arm22);
     
    }
    
    else if (counter==2400){
     arm1_motor->setPosition(position_arm11);
    }
    
    if(counter == 2600){
      arm_finish=false;
    }
    
    counter ++; 

  }
  color_difference= abs(front_color-bottom_color);
   
}

void line_following(Robot *robot,Motor *left_motor,Motor *right_motor,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,PositionSensor *left_ps,PositionSensor *right_ps){
    bool line_following= true;
    move_straight(robot,left_motor,right_motor,50);
    while ((robot->step(TIME_STEP) != -1) && line_following){
      if (left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
         single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      }
  
      else if (left2_ir_val==1 && rightcenter_ir_val==1 && right2_ir_val==0){
        turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
        
      }
  
      else if(right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0 ) {
      
        turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
      }
      else{
        single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      
      }
      
      if(left2_ir_val==0 && left1_ir_val==0 && leftcenter_ir_val==0 && rightcenter_ir_val==0 && right1_ir_val==0 && right2_ir_val==0){
        line_following=false;
      
        }

}
}

void wall_following(Robot *robot,Motor *left_motor,Motor *right_motor, DistanceSensor *ds_right , DistanceSensor *ds_left, DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2 ){
  bool wall_follow = true;
  last_wr_error = 0;
  last_wl_error = 0;
  double left_speed;
  double right_speed;
  while ((robot->step(TIME_STEP) != -1) && wall_follow){
    double ds_right_val = ds_right->getValue();
    double ds_left_val = ds_left->getValue();
    
    //PID parametres for right motor 
    double r_position = ds_right_val;
    double wr_error = (r_position - target)/10;
    double wr_derivative = wr_error - last_wr_error;
    last_wr_error = wr_error;
   
   //PID parametres for left motor 
   
    double l_position = ds_left_val;
    double wl_error = (l_position - target)/10;
    double wl_derivative = wl_error - last_wl_error;
    last_wl_error = wl_error;
    
    
    
    
      if ((ds_right_val <950)&&(ds_left_val==1000)){
        left_speed=(base_speed+ 0.1*wr_error + 1*wr_derivative);
        right_speed=(base_speed-0.1*wr_error-1*wr_derivative);
        if(left_speed>30){
          left_speed=30; 
        }
        if(right_speed>30){
          right_speed=30; 
        }
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);
      }
      else if((ds_right_val ==1000)&&(ds_left_val<950)){
        left_speed=(base_speed-0.1*wl_error-1*wl_derivative);
        right_speed=(base_speed+0.1*wl_error+1*wl_derivative);
        if(left_speed>30){
          left_speed=30; 
        }
        if(right_speed>30){
          right_speed=30; 
        }
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);
      }
      
      
       ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
      if(left2_ir_val==1 || left1_ir_val==1 || leftcenter_ir_val==1 || rightcenter_ir_val==1 || right1_ir_val==1 || right2_ir_val==1){
        wall_follow=false;
      }
    
  }

}



void line_following_before_maze(Robot *robot,Motor *left_motor,Motor *right_motor,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,PositionSensor *left_ps,PositionSensor *right_ps){
    int no_of_rturn=0;
    bool line_following= true;
    while ((robot->step(TIME_STEP) != -1) && line_following){
    
     if(right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0 ) {
        if(no_of_rturn==2){
          line_following=false;
        }
        else{
          
          turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
        }
        no_of_rturn++;
      }
      else{
        single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      
      }
      
      if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
        line_following=false;
      
        }

    }
}




void maze_solving( Camera *cam,Robot *robot,Motor *left_motor,Motor *right_motor, DistanceSensor *ds_right , DistanceSensor *ds_left,DistanceSensor *ds_right_maze, DistanceSensor *ds_front, DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,PositionSensor *left_ps,PositionSensor *right_ps,Motor*r_motor,Motor*l_motor,Motor*base_motor,Motor*arm1_motor,Motor*arm2_motor){
  int stage=0;
  
  turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
  std::cout << " Quadrant No: 1 " <<std::endl;
  while((robot->step(TIME_STEP) != -1) && stage==0 ){
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if((right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0)){
    stage++;
    }
  }
  double distance = move_single(robot,left_motor,right_motor,ds_right_maze);
  
  if(distance<300){
   move_straight(robot,left_motor,right_motor,30);
    std::cout << " Quadrant No: 2 " <<std::endl;
    
   while((robot->step(TIME_STEP) != -1) && stage==1){ 
     single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
     if((right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0)){
      stage++;
      }
    }
    
   move_straight(robot,left_motor,right_motor,30);
   
   std::cout << " Quadrant No: 3 " <<std::endl;
   while((robot->step(TIME_STEP) != -1) && stage==2){ 
     single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
     if((right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0)){
      stage++;
      }
    }
   
   turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
   
   while((robot->step(TIME_STEP) != -1) && stage==3){ 
     single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
     double dis1 = ds_front->getValue();
     if(dis1<=100){
      stage++;
      stop(left_motor,right_motor);
      }
      
    }
    arm_function(cam,robot,r_motor,l_motor,base_motor,arm1_motor,arm2_motor);
    
   about_turn(robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2,7);
   
   turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
   
   while((robot->step(TIME_STEP) != -1) && stage==4){ 
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
     stage++;
     }
   }
   
   turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
   
    std::cout << " Quadrant No: 2" <<std::endl;
   while((robot->step(TIME_STEP) != -1) && stage==5){ 
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if(right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0){
     stage++;
     }
   }
   
   turn_right(0.68,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
   
  }
  
  else if(300< distance && distance <750){
    turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
    
    while((robot->step(TIME_STEP) != -1) && stage==1){ 
     single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
     double dis1 = ds_front->getValue();
     if(dis1<=100){
      stage++;
      stop(left_motor,right_motor);
      }
      
    }
    arm_function(cam,robot,r_motor,l_motor,base_motor,arm1_motor,arm2_motor);
    
    about_turn(robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2,7);
    
    move_straight(robot,left_motor,right_motor,30);
    
    while((robot->step(TIME_STEP) != -1) && stage==2){ 
      single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
       stage++;
       }
    }
    
    turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
    
     std::cout << " Quadrant No: 2 " <<std::endl;
    while((robot->step(TIME_STEP) != -1) && stage==3){ 
      single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      if(left2_ir_val==1 && rightcenter_ir_val==1 &&  right2_ir_val==0){
       stage++;
       }
    }
    
    turn_left(0.68,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
    
  }
  
  else{
    
    move_straight(robot,left_motor,right_motor,30);
    
     std::cout << " Quadrant No: 2 " <<std::endl;
    while((robot->step(TIME_STEP) != -1) && stage==1){ 
      single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
      if(right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0){
       stage++;
       }
    }
     double distance1 = move_single(robot,left_motor,right_motor,ds_right_maze);
     
     if(distance1<300){
       move_straight(robot,left_motor,right_motor,30);
       
       std::cout << " Quadrant No: 3 " <<std::endl;
       while((robot->step(TIME_STEP) != -1) && stage==2){ 
          single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
         if((right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0)){
          stage++;
          }
       }
       
       move_straight(robot,left_motor,right_motor,30);
       std::cout << " Quadrant No: 4 " <<std::endl;
      
       while((robot->step(TIME_STEP) != -1) && stage==3){ 
         single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
         if((right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0)){
          stage++;
          }
        }
   
       turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
   
       while((robot->step(TIME_STEP) != -1) && stage==4){ 
         single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
         double dis1 = ds_front->getValue();
         if(dis1<=100){
          stage++;
          stop(left_motor,right_motor);
          }
      
        }
        
        arm_function(cam,robot,r_motor,l_motor,base_motor,arm1_motor,arm2_motor);
    
       about_turn(robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2,7.5);
       
       turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
       
       while((robot->step(TIME_STEP) != -1) && stage==5){ 
        single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
        if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
         stage++;
         }
        }
        
        turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
        
        std::cout << " Quadrant No: 2 " <<std::endl;
        while((robot->step(TIME_STEP) != -1) && stage==6){ 
          single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
          if(left2_ir_val==1 && rightcenter_ir_val==1 &&  right2_ir_val==0){
           stage++;
           }
        }
        
        turn_left(0.68,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
        
        
       }
       else if (300< distance1 && distance1 < 750){
         turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
         
         while((robot->step(TIME_STEP) != -1) && stage==2){ 
           single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
           double dis1 = ds_front->getValue();
           //std::cout << "ds_front: " << dis1 <<std::endl;
           if(dis1<=100){  
            stage++;
            stop(left_motor,right_motor);
            }
            
          }
          
          arm_function(cam,robot,r_motor,l_motor,base_motor,arm1_motor,arm2_motor);
          
          about_turn(robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2,7);
          
          move_straight(robot,left_motor,right_motor,30);
          
          while((robot->step(TIME_STEP) != -1) && stage==3){ 
            single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
            if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
             stage++;
             }
          }
          
          turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
          
           std::cout << " Quadrant No: 2 " <<std::endl;
           while((robot->step(TIME_STEP) != -1) && stage==4){ 
            single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
            if(right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val==0){
             stage++;
             }
           }
           
           turn_right(0.68,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
          
       }
       
  }
  
  
}

void dash_line_following(Robot *robot,Motor *left_motor,Motor *right_motor,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,PositionSensor *left_ps,PositionSensor *right_ps,DistanceSensor *ds_front,InertialUnit *imu ){
    
    bool line_following= true;
    bool ramp_detect= true;
    while ((robot->step(TIME_STEP) != -1)&&ramp_detect){
      
      while ((robot->step(TIME_STEP) != -1) && line_following){
        ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
        double P= get_error();
        double D = P - final_error;
        final_error = P;
        left_motor->setVelocity(5+800*P+200*D);
        right_motor->setVelocity(5-800*P-200*D);
        const double *values = imu->getRollPitchYaw();
    if (values[1] > 0.1){
      return;

    }
        if(left2_ir_val==0 && left1_ir_val==0 && leftcenter_ir_val==0 && rightcenter_ir_val==0 && right1_ir_val==0 && right2_ir_val==0){
          line_following= false;
        }
      }
      while((robot->step(TIME_STEP) != -1) && line_following==false ){
        ir_update(irl2,irl1,ircl,ircr,irr1,irr2);
       
        if(left2_ir_val==1 || left1_ir_val==1 || leftcenter_ir_val==1 || rightcenter_ir_val==1 || right1_ir_val==1 || right2_ir_val==1){
          line_following= true;
         
        }
      }
        
     }
}

void ramp(Robot *robot,Motor *left_motor,Motor *right_motor,PositionSensor *left_ps,PositionSensor *right_ps,DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){  
    base_speed=7;
    while (robot->step(TIME_STEP) != -1 ){
        single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
        if (left2_ir_val==1 && right2_ir_val==1){
            base_speed=5;
            if (color_difference%2==0){
                turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
                return;
            }

            else{
                turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
                return;
            }
        }
    }
}


void pillarcount(Robot *robot,Motor *left_motor,Motor *right_motor, DistanceSensor *ds_right , DistanceSensor *ds_left, DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2,PositionSensor *left_ps,PositionSensor *right_ps){
    int pillar_c=0;
    double ds_right_val;
    double ds_left_val;
    double pre_r=1000;
    double pre_l=1000;
    int thresh_dis=650;   
    
    while((robot->step(TIME_STEP) != -1) ){ 
        single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
        
        ds_right_val = ds_right->getValue();
        ds_left_val = ds_left->getValue();
        
        if (color_difference %2==1){		//odd color_num

            if ((ds_right_val<thresh_dis) && (pre_r>thresh_dis)){
                //std::cout << "pillar count " <<pillar_c<< std::endl;
                pillar_c+=1;
            }
            pre_r=ds_right_val;
            
            //checking the path
            if (right2_ir_val==1 && leftcenter_ir_val==1 && left2_ir_val ==0 ){
                std::cout << "Total number of pillars:  " <<pillar_c<< std::endl;
    
                if (pillar_c==2){
                     std::cout << "Path is correct " << std::endl;
                }
                else{
                     std::cout << "Path is wrong" << std::endl;
                }
                turn_right(0.7025,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
                break;
            }
           
        }

        else{		//even color_num
	 
            if ((ds_left_val<thresh_dis) && (pre_l>thresh_dis)){
                pillar_c+=1;
                //std::cout << "pillar count " <<pillar_c<< std::endl;
            }
            pre_l=ds_left_val;
            
            //checking the path
            if (left2_ir_val==1 && rightcenter_ir_val==1 && right2_ir_val==0 ){
                std::cout << "Total number of pillars:  " <<pillar_c<< std::endl;
    
                if (pillar_c==2){
                     std::cout << "Path is correct " << std::endl;
                }
                else{
                     std::cout << "Path is wrong" << std::endl;
                }
                turn_left(0.7,robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
                break;
            }
            
        }

     }
}




void gate_solving(Robot *robot,Motor*left_motor,Motor*right_motor,DistanceSensor *ds_front, DistanceSensor *irl2,DistanceSensor *irl1,DistanceSensor *ircl,DistanceSensor *ircr,DistanceSensor *irr1,DistanceSensor *irr2){
  int g_stage =0;
  bool g1_open= true;
  bool g2_open=true;
  int thres=300;
  double g_dist=0;
  while((robot->step(TIME_STEP) != -1) && g_stage==0){ 
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
       g_stage++;
     }
  }
  
  stop(left_motor,right_motor);   
  while((robot->step(TIME_STEP) != -1) && g1_open){
    g_dist = ds_front->getValue();
    if(g_dist<thres){
      g1_open=false;
    }
  }
  while((robot->step(TIME_STEP) != -1) && !(g1_open)){
    g_dist = ds_front->getValue();
    if(g_dist>thres){
      g1_open=true;
    }
  }
  move_straight(robot,left_motor,right_motor,30);
  while((robot->step(TIME_STEP) != -1) && g_stage==1){ 
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
       g_stage++;
     }
  }
  
  stop(left_motor,right_motor);  
  while((robot->step(TIME_STEP) != -1) && !(g2_open)){
    g_dist = ds_front->getValue();
    if(g_dist>thres){
      g2_open=true;
    }
  }
  move_straight(robot,left_motor,right_motor,30);
  while((robot->step(TIME_STEP) != -1) && g_stage==2){ 
    single_linefollow(left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2);
    if(left2_ir_val==1 && left1_ir_val==1 && leftcenter_ir_val==1 && rightcenter_ir_val==1 && right1_ir_val==1 && right2_ir_val==1){
       g_stage++;
     }
  }
  move_straight(robot,left_motor,right_motor,70);
  stop(left_motor,right_motor);  
}



int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  //motors
  Motor *left_motor = robot->getMotor("motor_1");
  Motor *right_motor = robot->getMotor("motor_2");
  
  Motor*r_motor = robot->getMotor("r_motor");
  Motor*l_motor = robot->getMotor("l_motor");
  Motor*base_motor = robot->getMotor("base_motor");
  Motor*arm1_motor = robot->getMotor("arm1_motor");
  Motor*arm2_motor = robot->getMotor("arm2_motor");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  
  //laser sensors
  DistanceSensor *ds_right = robot->getDistanceSensor("ds_right");
  ds_right->enable(TIME_STEP);
  DistanceSensor *ds_left = robot->getDistanceSensor("ds_left");
  ds_left->enable(TIME_STEP);
  DistanceSensor *ds_front = robot->getDistanceSensor("ds_front");
  ds_front->enable(TIME_STEP);
  DistanceSensor *ds_right_maze = robot->getDistanceSensor("ds_right_maze");
  ds_right_maze->enable(TIME_STEP);

  //ir sensors
  DistanceSensor *irl2 = robot-> getDistanceSensor("IRL2");
  irl2->enable(TIME_STEP);
  DistanceSensor *irl1 = robot-> getDistanceSensor("IRL1");
  irl1->enable(TIME_STEP);
  DistanceSensor *ircl = robot-> getDistanceSensor("IRCL");
  ircl->enable(TIME_STEP);
  DistanceSensor *ircr = robot-> getDistanceSensor("IRCR");
  ircr->enable(TIME_STEP);
  DistanceSensor *irr1 = robot-> getDistanceSensor("IRR1");
  irr1->enable(TIME_STEP);
  DistanceSensor *irr2 = robot-> getDistanceSensor("IRR2");
  irr2->enable(TIME_STEP);

  //Create position sensor instances
  PositionSensor *left_ps=robot->getPositionSensor("ps_1");
  left_ps->enable(TIME_STEP);
  PositionSensor *right_ps=robot->getPositionSensor("ps_2");
  right_ps->enable(TIME_STEP);
  
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  imu->enable(TIME_STEP);
   
  // intial position of arm
  base_motor->setPosition(position_base1);
  arm1_motor->setPosition(position_arm11);
  arm2_motor->setPosition(position_arm22);
   
  Camera *cam = robot->getCamera("CAM");

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
 
  line_following(robot,left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2,left_ps,right_ps);
  
  wall_following(robot,left_motor,right_motor,ds_right ,ds_left,irl2,irl1,ircl,ircr,irr1,irr2 );
  
  line_following_before_maze(robot,left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2,left_ps,right_ps);
  
  maze_solving(cam,robot,left_motor,right_motor,ds_right ,ds_left,ds_right_maze, ds_front,irl2,irl1,ircl,ircr,irr1,irr2,left_ps,right_ps,r_motor,l_motor,base_motor,arm1_motor,arm2_motor );
  
  dash_line_following(robot,left_motor,right_motor,irl2,irl1,ircl,ircr,irr1,irr2,left_ps,right_ps,ds_front,imu );
  
  ramp(robot,left_motor,right_motor,left_ps,right_ps,irl2,irl1,ircl,ircr,irr1,irr2);
  
  pillarcount(robot,left_motor,right_motor,ds_right ,ds_left,irl2,irl1,ircl,ircr,irr1,irr2,left_ps,right_ps);
  
  gate_solving(robot,left_motor,right_motor,ds_front,irl2,irl1,ircl,ircr,irr1,irr2);
  
  delete robot;
  return 0;

}
