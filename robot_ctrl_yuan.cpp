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

void Robot::Robot2Ball(int n, double last[], double integral[], int *flag1, int*flag2,int *flag3, double *radius ,double kick[])
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
    double kki=9.5;

    // jihe coefficients
    double x_l=5.2;  // the location of the door
    double R;
    double k1,k2;
    double a,b;
    double x0,y0;
    double expected_rot;
    double e1=0.0015;

    double ball_xx,ball_yy;
    double theta1;

    //initialize the last condition
    if (n == 1) 
    {
      last[1] = ball_x;
      last[2] = ball_y;
      last[3] = robot_x;
      last[4] = robot_y;
      last[5] = robot_rot;
      *flag1=0;
    }

    //calculate the kick location
    theta1=atan(-ball_y/(x_l-ball_x));
    ball_xx=ball_x-0.015*cos(theta1);
    ball_yy=ball_y-0.015*sin(theta1);
    //calculate the rotation
    // 计算斜率
    k1 = (ball_xx - robot_x) / (ball_yy - robot_y) ;
    k2 = (ball_xx -x_l) / ball_yy;
    // 计算分子、分母项
    a=ball_xx * k2 - (robot_x + ball_xx)/2*k1 + ball_yy - (robot_y + ball_yy)/2;
    b=k1-k2;
    // 计算圆心
    x0=-a/b;
    y0=-(x0 - ball_xx)*k2 + ball_yy;
    // calculate expected rotation
    expected_rot=PI/2+atan((robot_y - y0) / (robot_x - x0));
    //whether to kick?
    if ( fabs(expected_rot-robot_rot) <= e1 || *flag1==1)
    {
        // kick
        ROS_INFO_STREAM( "KICK" << *flag1);
        if (sqrt( pow((last[1] - ball_x),2) + pow((last[2] - ball_y),2) )<=e1)
        {
            velocity=0.20;
            angular= -velocity/(*radius);
            *flag1=1;
        }
        else
        {
            velocity=0.20;
            angular= 0;
            *flag1=1;
        }
    }
    else
    {
        // calculate radius
        *radius=sqrt( pow((x0 - ball_xx),2) + pow((y0 - ball_yy),2) ) ;
        // tiao theta
        ROS_INFO_STREAM( "TIAO EXPECTED_ROT" );
        velocity=0;
        angular=kki * ( expected_rot - robot_rot ) + kkp/T * ( last[5] - robot_rot );
    }
    
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
    double radius=1;

    ros::init(argc, argv, "robot_ctrl");

    Robot t1;
    ros::Rate rate(30);

    while(ros::ok())
    {
        t1.Robot2Ball(n,last,integral,&flag1,&flag2,&flag3,&radius,kick);
        ros::spinOnce();
        rate.sleep();

        n=n+1;
    }

    //ros::spin();
    return 0;
}
