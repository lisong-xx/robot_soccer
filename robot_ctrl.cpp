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
double orientation(int n,double robot_rot, double expected_rot )
{
    double robot_rot_last;
    double angular;
    double kp=0.04;
    double kd=0.02;
    if (n == 1)
    {
       robot_rot_last = 0;
       angular=kp*(robot_rot-expected_rot)+kd*(robot_rot-robot_rot_last);
    }
    else
    {
       angular=kp*(robot_rot-expected_rot)+kd*(robot_rot-robot_rot_last);
       robot_rot_last = robot_rot;
    }
return angular;
}
void Robot::Robot2Ball(int n)
{
    double velocity;
    double angular;

    // ball pos and robot pos
    double ball_x = position_ball.x;
    double ball_y = position_ball.y;
    double robot_x = position_robot.x;
    double robot_y = position_robot.y;
    double robot_rot = orientation_robot;
    // Todo: write you code here
    double x_l=10.5;
    double R;
    double k1,k2;
    double x0,y0;
    double expected_rot;

    double ball_x_last,ball_y_last;
    double e=0.01;
    
    if (n == 1) 
    {
      ball_x_last = ball_x;
      ball_y_last =ball_y;
    }
    if ( sqrt( pow((ball_x_last-ball_x),2) + pow((ball_y_last-ball_y),2) ) <= e )
    {
      if (ball_y != 0 && (ball_x-robot_x) != 0)
      {
         k1 = (ball_y - robot_y) / (ball_x - robot_x);
         k2 = (ball_x -x_l) / ball_y;
         if (k1 != k2)
         {
           a=ball_x * k2 - (robot_x + ball_x)/2*k1 + ball_y - (robot_y + ball_y)/2;
           b=k1-k2;
         
         x0=-a/b;
         y0=-(x0 - ball_x)*k2 + ball_y;
         
         R=sqrt( pow((x0 - ball_x),2) + pow((y0 - ball_y),2) );
         
         expected_rot=PI/2+atan((robot_y - y0) / (robot_x - x0));
         if ( sqrt( pow((expected_rot-robot_rot),2) ) <= e )
           {
              velocity=0.8;
              angular=
           }
         else
           {
              
           }
         }
         else
         {
          }
      }
      else
      {
      }
       
       
    }
    else
    {
      velocity=0;
      angular=0;
    }

    ball_x_last=ball_x;
    ball_y_last=ball_y;
    RobotTwistPub(velocity,angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_ctrl");
    int n=1;
    Robot t1;
    ros::Rate rate(30);

    while(ros::ok())
    {
        t1.Robot2Ball(n);
        ros::spinOnce();
        rate.sleep();
        n=n+1;
    }

    //ros::spin();
    return 0;
}
