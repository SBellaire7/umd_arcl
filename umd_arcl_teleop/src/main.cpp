#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class ArclTeleop
{
  private:

    //handles
    ros::Subscriber joySub;
    ros::Publisher twistPub;

  public:

    ArclTeleop(ros::NodeHandle* nh)
    {
      //setup subscribers and publishers
      joySub = nh->subscribe("/joy", 1, &ArclTeleop::joySubCB, this);
      twistPub = nh->advertise<geometry_msgs::Twist>("arcl/cmd_vel", 1);
    }

    void joySubCB(const sensor_msgs::Joy& msg)
    {
      //get deadman switches
      bool dmSw = msg.buttons[4] | msg.buttons[5];

      //setup message (with 0s if dm sw isnt pressed)
      geometry_msgs::Twist twistMsg;
      if(dmSw) twistMsg.linear.x = msg.axes[1] * 1000;
      else twistMsg.linear.x = 0;
      twistMsg.linear.y = 0;
      twistMsg.linear.z = 0;
      twistMsg.angular.x = 0;
      twistMsg.angular.y = 0;
      if(dmSw) twistMsg.angular.z = msg.axes[3] * 100;
      else twistMsg.angular.z = 0;

      //send msg
      twistPub.publish(twistMsg);
    }
};

int main(int argc, char** argv)
{
  //init ros
  ros::init(argc, argv, "arclTeleop");
  ros::NodeHandle nh;
  ArclTeleop Teleop(&nh);

  //let ros take over
  ros::Rate rate(2);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

