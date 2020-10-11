#include "ros/ros.h"
#include "robot_ctrl/robot_ctrl.h"
#include "geometry_msgs/Quaternion.h" 
#include "geometry_msgs/Point.h" 
#include "tf/transform_datatypes.h"//转换函数头文件 
#include <cmath>

// max vel: 1.2m/s
// max ang:10.0rad/s

Robot::Robot()
{  
    index_ball = index_robot = -1;
    std::string topic_model_state = "/gazebo/model_states";

    nh.param("ball_name",ball_name,std::string("ball0"));
   
    ROS_INFO("ball_name: %s",ball_name.c_str());

    sub_model_pos = nh.subscribe(topic_model_state, 10, &Robot::ModelStateCallback, this);

    pub_robot_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void Robot::ModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &cur_state)
{
    // int box_index = -1;
    std::vector<std::string> model_names = cur_state->name;
    if(index_ball<0||index_robot<0)
    {
        ROS_INFO("update_index");
        for(size_t i = 0; i < model_names.size(); i++)
        {
            if(model_names[i] == ball_name)
                index_ball = i;
            else if(model_names[i] == "home1")
                index_robot = i;
        }
    }

    geometry_msgs::Pose pose_ball;
    geometry_msgs::Pose pose_robot;
    geometry_msgs::Quaternion quat_robot;
    pose_ball = cur_state->pose[index_ball];
    pose_robot = cur_state->pose[index_robot];
    position_ball = pose_ball.position;
    position_robot = pose_robot.position;
    quat_robot = pose_robot.orientation;

    //ROS_INFO("bx: %.03f, by: %.03f",position_ball.x,position_ball.y);
    //ROS_INFO("rx: %.03f, ry: %.03f",position_robot.x,position_robot.y);

    // convert quat 2 yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_robot, quat);
 
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    orientation_robot = yaw;


    //ROS_INFO_STREAM("orientation:"  << yaw);
}

void Robot::RobotTwistPub(double velocity,double angular)
{
    if(velocity > MAX_LIN_VEL) velocity = MAX_LIN_VEL;
    if(angular > MAX_ANG_VEL) angular = MAX_ANG_VEL;
    //ROS_INFO("linear: %.03f, angular: %.03f",velocity,angular);
    geometry_msgs::Twist vel;
    vel.linear.x = velocity;
    vel.angular.z = angular;
    pub_robot_vel.publish(vel);
}

void Robot::Robot2Ball(int n, double last[], double integral[], int *flag1, int*flag2,int *flag3,double *radius, double kick[])
{
    double velocity;
    double angular;
    
    // ball pos and robot pos
    double ball_x = position_ball.x;
    double ball_y = position_ball.y;
    double robot_x = position_robot.x;
    double robot_y = position_robot.y;
    double robot_rot = orientation_robot;
    // PID coefficients
    double T=1.0/30.0;
    double ki=0.1;  
    double kp=0.01;
    double kkp=0.5;
    double kki=6.5;
    // jihe coefficients
    double K=0.15;
    double theta1,theta2;
    double x_l=5.5;
    double e1=0.002;
    double e2=0.003;
    double a,b;

    // Todo: write you code here
    // initialize   
    if (n == 1) 
    {
      last[1] = ball_x;
      last[2] = ball_y;
      last[3] = robot_x;
      last[4] = robot_y;
      last[5] = robot_rot;
    } 
    // whether to move
    //if (sqrt( pow((last[1] - ball_x),2) + pow((last[2] - ball_y),2)) >= 100*e2)
    //{
    //   ROS_INFO_STREAM( "STOP" );
       //stop the robot
    //   velocity=0;
    //   angular=0;
    //}
    //else
    //{
       //execute the program
      if ((ball_x-0.1) <= robot_x && *flag2==0)
      {
        if ((ball_y-0.2) <= robot_y && (ball_y+0.2) >= robot_y)
        {
          // bikai
          if (fabs(PI/2-robot_rot) <= 10*e1 )
          {  
             // houtui
             velocity=0.25;
             angular=0.1*kki * (PI/2 - robot_rot) + 0.1*kkp/T * (last[5] - robot_rot);
          }
          // TIAO theta
          else 
          {  
              // rotate
              velocity=0;
              angular=kki * (PI/2 - robot_rot) + kkp/T * (last[5] - robot_rot);
           }
        }
        else
        {
          // move robot to the rear of the ball
          if (fabs(robot_rot) <= 10*e1 )
          {  
             // houtui
             velocity=-0.3;
             angular=0.1*kki * (0 - robot_rot) + 0.1*kkp/T * (last[5] - robot_rot);
          }
          // TIAO theta
          else 
          {  
              // rotate
              velocity=0;
              angular=kki * (0 - robot_rot) + kkp/T * (last[5] - robot_rot);
          }
        }
      }
      else
      {
        // kick the ball

        //caculate kick location
        theta1=atan(-ball_y/(x_l-ball_x));
        kick[1]=ball_x-K*cos(theta1);
        kick[2]=ball_y-K*sin(theta1);
  
        //caculate kick angle
        if (kick[1] <= robot_x && kick[2] <= robot_y)
        {
           theta2=-PI+atan((kick[2]-robot_y) / (kick[1]-robot_x));
        }
        else if (kick[1] <= robot_x)
        {
           theta2=PI+atan((kick[2]-robot_y) / (kick[1]-robot_x));
        }
        else
        {
           theta2=atan((kick[2]-robot_y) / (kick[1]-robot_x));
        }
    
    
        // quanli or quanwai
        if ( sqrt( pow((kick[1] - robot_x),2) + pow((kick[2] - robot_y),2)) <= e2 || *flag1==1)
        {  
           ROS_INFO_STREAM( "QUANLI" );
           
           // theta1 is ok? 
           if (fabs(theta1-robot_rot)<=0.25*e1 || *flag2==1)
           {  
              ROS_INFO_STREAM("KICK");
              // kick
              velocity=0.3;
              angular=0.1*kki * (theta1 - robot_rot) + 0.1*kkp/T * (last[5] - robot_rot);;
              *flag2=1;
           }
           // TIAO THETA1  
           else 
           {  
              ROS_INFO_STREAM("QUANLI TIAO THETA1");
              // rotate
              velocity=0;
              angular=kki * (theta1 - robot_rot) + kkp/T * (last[5] - robot_rot);
           }
    
           *flag1=1;
    
        }
        // quanwai
        else
        {
           ROS_INFO_STREAM("QUANWAI");
           // theta2 is ok?
           if (fabs(theta2-robot_rot) <= 20*e1 )
           {
              ROS_INFO_STREAM("QUANWAI tiao R");
              //move
              a=sqrt( pow((kick[1] - robot_x),2) + pow((kick[2] - robot_y),2) );
              b=sqrt( pow((last[3] - robot_x),2) + pow((last[4] - robot_y),2) );
    
              velocity= ki * a + kp/T * (a - b);
              angular = kki * (theta2 - robot_rot) + kkp/T * (last[5] - robot_rot);
           }
           // tiao theta2
           else 
           {
              ROS_INFO_STREAM("QUANWAI tiao theta2");
              // rotate to proper rotation
              velocity= 0;
              angular = kki * (theta2 - robot_rot) + kkp/T * (last[5] - robot_rot);
           }  
        }
      }// whether behind the ball
    //}//whether to move  
    //remember the last time condition.
    last[1] = ball_x;
    last[2] = ball_y;
    last[3] = robot_x;
    last[4] = robot_y;
    last[5] = robot_rot;

    //publish
    RobotTwistPub(velocity,angular);

}

int main(int argc, char **argv)
{
    int n=1;
    int flag1=0;
    int flag2=0;
    int flag3=0;
    double last[5];
    double integral[5];
    double kick[2];
    double radius;

    ros::init(argc, argv, "robot_ctrl");

    Robot t1;
    ros::Rate rate(30);

    while(ros::ok())
    {
        t1.Robot2Ball(n,last,integral,&flag1,&flag2,&flag3,&radius,kick);
        ros::spinOnce();
        rate.sleep();
        //ROS_INFO_STREAM ("flag1="<<flag1);
        n=n+1;
    }

    //ros::spin();
    return 0;
}
