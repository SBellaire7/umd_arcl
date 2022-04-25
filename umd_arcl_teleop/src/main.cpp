#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>

class ArclTeleop
{
  private:

    //handles
    ros::Subscriber joySub;
    ros::Publisher movePub;
    ros::Publisher rotatePub;
    ros::Publisher gsPub;

    bool dmSw;
    int moveSpd;
    int turnSpd;
    int prevMoveSpd;
    int prevTurnSpd;

  public:

    ArclTeleop(ros::NodeHandle* nh)
    {
      joySub = nh->subscribe("/joy", 1, &ArclTeleop::joySubCB, this);
      movePub = nh->advertise<std_msgs::Int32MultiArray>
        ("arcl/move", 1);
      rotatePub = nh->advertise<std_msgs::Int32MultiArray>
        ("arcl/rotate", 1);
      gsPub = nh->advertise<std_msgs::Bool>("arcl/goStop", 1);

      dmSw = false;
      moveSpd = 0;
      turnSpd = 0;
      prevMoveSpd = 0;
      prevTurnSpd = 0;
    }

    void joySubCB(const sensor_msgs::Joy& msg)
    {
      //get left joy (fwd vel) and right joy (turning)
      float axMove = msg.axes[1];
      float axTurn = msg.axes[3];

      //get deadman switch (either LB or RB)
      dmSw = msg.buttons[4] | msg.buttons[5];

      //calculate velocities (max 1m/s or 100deg/s)
      moveSpd = axMove * 1000;
      turnSpd = axTurn * 100;

      //clamp values
      if(moveSpd > 1000) moveSpd = 1000;
      if(moveSpd < -1000) moveSpd = -1000;
      if(turnSpd > 100) turnSpd = 100;
      if(turnSpd < -100) turnSpd = -100;
      if(abs(moveSpd) < 20) moveSpd = 0;
      if(abs(turnSpd) < 5) turnSpd = 0;

      //pub right away if dms is released
      if(!dmSw)
      {
        arclPub();
      }
    }

    void arclPub()
    {
      //create msg and clear out data
      std_msgs::Bool gsMsg;
      std_msgs::Int32MultiArray msg;
      msg.data.clear();

      //if turn and move are close to 0 or dmSw not pressed,
      //stop the robot from turning
      if(((abs(turnSpd) == 0) && (abs(moveSpd) == 0)) || !dmSw)
      {
        prevMoveSpd = 0;
        prevMoveSpd = 0;
        gsMsg.data = 0;
        gsPub.publish(gsMsg);
      }

      //otherwise prioritize turning
      else if((abs(turnSpd) > 0) && (abs(turnSpd - prevTurnSpd) > 5))
      {
        prevTurnSpd = turnSpd;
        gsMsg.data = 1;
        gsPub.publish(gsMsg);
        if(turnSpd > 0) msg.data.push_back(1440);
        else msg.data.push_back(-1440);
        msg.data.push_back(abs(turnSpd));
        rotatePub.publish(msg);
      }

      //otherwise move
      else if(abs(prevMoveSpd - moveSpd) > 20)
      {
        prevMoveSpd = moveSpd;
        gsMsg.data = 1;
        gsPub.publish(gsMsg);
        if(moveSpd > 0) msg.data.push_back(10000);
        else msg.data.push_back(-10000);
        msg.data.push_back(abs(moveSpd));
        movePub.publish(msg);
      }
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
    Teleop.arclPub();
    rate.sleep();
  }
}
