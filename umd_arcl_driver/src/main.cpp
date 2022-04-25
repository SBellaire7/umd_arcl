#include <ros/ros.h>
#include <string>
#include <sys/time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <umd_arcl_driver/Pose2DStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

#include "../include/arclUtil.h"

class ArclDriver
{
  private:

    //subscriber handles
    ros::Subscriber moveSub;
    ros::Subscriber rotateSub;
    ros::Subscriber rotateToSub;
    ros::Subscriber gsSub;
    ros::Subscriber saySub;
    ros::Subscriber cmdVelSub;

    //publisher handles
    ros::Publisher socPub;
    ros::Publisher posePub;
    ros::Publisher lidarPub;
    ros::Publisher lidarLowPub;

    //arcl connection vars
    int port, sfd;
    std::string ip, pw;

    //ros params and other vars
    int statusRate, lidarRate;
    Point2DArr pArr;
    std::string primaryLidarName, lowLidarName;

  public:

    ArclDriver(ros::NodeHandle* nh)
    {
      //init point array size to 0
      pArr.size = 0;

      //get parameters
      if(!nh->hasParam("arcl/ip")) ROS_WARN_STREAM("IP parameter not found. Using default\n");
      ros::param::param<std::string>("arcl/ip", ip, "192.168.1.61");
      if(!nh->hasParam("arcl/port")) ROS_WARN_STREAM("Port parameter not found. Using default\n");
      ros::param::param<int>("arcl/port", port, 7171);
      if(!nh->hasParam("arcl/pw")) ROS_WARN_STREAM("Password parameter not found. Using default\n");
      ros::param::param<std::string>("arcl/pw", pw, "omron");
      if(!nh->hasParam("arcl/statusRate")) ROS_WARN_STREAM("Status request rate parameter not found. Using default\n");
      ros::param::param<int>("arcl/statusRate", statusRate, 10);
      if(!nh->hasParam("arcl/lidarRate")) ROS_WARN_STREAM("Lidar request rate parameter not found. Using default\n");
      ros::param::param<int>("arcl/lidarRate", lidarRate, 1);
      if(!nh->hasParam("arcl/primaryLidarName")) ROS_WARN_STREAM("Primary lidar name parameter not found. Using default\n");
      ros::param::param<std::string>("arcl/primaryLidarName", primaryLidarName, "Laser_1");
      if(!nh->hasParam("arcl/lowLidarName")) ROS_WARN_STREAM("Low lidar name parameter not found. Using default\n");
      ros::param::param<std::string>("arcl/lowLidarName", lowLidarName, "Laser_2");

      //login to arcl server
      sfd = arclLogin(ip.c_str(), port, pw.c_str());
      if(sfd < 0)
      {
        if(sfd > -4) ROS_FATAL("Error connecting to %s. Aborting with code %d\n", ip.c_str(), sfd);
        else if(sfd == -4) ROS_FATAL("Connection to %s rejected by remote host\n", ip.c_str());
        exit(-1);
      }
      ROS_INFO("Connected to ARCL server %s\n", ip.c_str());

      //setup subscribers
      moveSub = nh->subscribe("arcl/move", 1, &ArclDriver::moveCB, this);
      rotateSub = nh->subscribe("arcl/rotate", 1, &ArclDriver::rotateCB, this);
      rotateToSub = nh->subscribe("arcl/rotateTo", 1, &ArclDriver:: rotateToCB, this);
      gsSub = nh->subscribe("arcl/goStop", 1, &ArclDriver::goStopCB, this);
      saySub = nh->subscribe("arcl/say", 1, &ArclDriver::sayCB, this);
      cmdVelSub = nh->subscribe("arcl/cmd_vel", 1, &ArclDriver::cmdVelCB, this);

      //setup publishers
      socPub = nh->advertise<std_msgs::Float32>("arcl/battery", 1);
      posePub = nh->advertise<umd_arcl_driver::Pose2DStamped>("arcl/pose", 1);
      lidarPub = nh->advertise<sensor_msgs::PointCloud>("arcl/lidar", 1);
      lidarLowPub = nh->advertise<sensor_msgs::PointCloud>("arcl/lidarLow", 1);
    }

    void moveCB(const std_msgs::Int32MultiArray& msg)
    {
      std::string cmd = "dotask move ";
      cmd = cmd + std::to_string(msg.data[0]) + " "
                + std::to_string(msg.data[1]) + "\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Move task was unsuccessful\n");
    }

    void rotateCB(const std_msgs::Int32MultiArray& msg)
    {
      std::string cmd = "dotask deltaheading ";
      cmd = cmd + std::to_string(msg.data[0]) + " "
                + std::to_string(msg.data[1]) + "\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Rotate task was unsuccessful\n");
    }

    void rotateToCB(const std_msgs::Int32MultiArray& msg)
    {
      std::string cmd = "dotask setheading ";
      cmd = cmd + std::to_string(msg.data[0]) + " "
                + std::to_string(msg.data[1]) + "\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Rotate-to task was unsuccessful\n");
    }

    void goStopCB(const std_msgs::Bool& msg)
    {
      std::string cmd;
      if(msg.data) cmd = "go\n";
      else cmd = "stop\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Go/Stop task was unsuccessful\n");
    }

    void sayCB(const std_msgs::String& msg)
    {
      std::string cmd = "say ";
      cmd = cmd + "\"" + msg.data + "\"\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Say command was unsuccessful\n");
    }

    void cmdVelCB(const geometry_msgs::Twist& msg)
    {
      //keep previous vals to smooth movement by not refreshing command if new val is similar
      static int prevLin = 0;
      static int prevAng = 0;

      int lin = msg.linear.x;
      int ang = msg.angular.z;

      //clamp values
      if(lin > 1000) lin = 1000;
      else if(lin < -1000) lin = -1000;
      if(ang > 90) ang = 90;
      else if(ang < -90) ang = -90;

      //implement deadzones
      if(abs(lin) < 20) lin = 0;
      if(abs(ang) < 5) ang = 0;

      //prioritize turning
      if(abs(ang) > 0)
      {
        //do not refresh cmd if new val is similar
        if(abs(ang - prevAng) <= 5) return;

        //construct turn msg
        std_msgs::Int32MultiArray msg;
        msg.data.clear();
        if(ang > 0) msg.data.push_back(1440);
        else msg.data.push_back(-1440);
        msg.data.push_back(abs(ang));

        //manually call rotate callback
        rotateCB(msg);

        //save previous value
        prevAng = ang;

        return;
      }

      //otherwise drive forward
      else if(abs(lin) > 0)
      {
        //do not refresh cmd if new val is similar
        if(abs(lin - prevLin) <= 20) return;

        //construct move msg
        std_msgs::Int32MultiArray msg;
        msg.data.clear();
        if(lin > 0) msg.data.push_back(10000);
        else msg.data.push_back(-10000);
        msg.data.push_back(abs(lin));

        //manually call move callback
        moveCB(msg);

        //save previous value
        prevLin = lin;

        return;
      }
    }

    void readLine()
    {
      //read line
      char buf[65536];
      int nb = socketReceiveLine(sfd, buf, 65536);

      //if nothing was read or an error occurred
      if(nb <= 0)
      {
        if(nb < 0) ROS_WARN_STREAM("Failed to read line from ARCL Server\n");
        return;
      }

      //check line for status keyword
      if(strstr(buf, "Status") != NULL)
      {
        //get battery % and pose
        float* f = arclParseStatus(buf);

        //check for nullptr
        if(f == NULL)
        {
          ROS_WARN_STREAM("Failed to fetch status info\n");
          return;
        }

        //publish
        pubStatus(f);

        //free memory
        free(f);
      }

      //check line for lidar keyword
      if(strstr(buf, "RangeDeviceGetCurrent") != NULL)
      {
        //check which laser the data belongs to
        int dev = 0;
        if(strstr(buf, primaryLidarName.c_str()) != NULL) dev = 1;
        else if(strstr(buf, lowLidarName.c_str()) != NULL) dev = 2;

        //get list of points
        arclParseLidar(buf, &pArr);

        //check for null
        if(pArr.size == 0)
        {
          ROS_WARN_STREAM("Failed to fetch current lidar data\n");
          return;
        }

        //publish
        pubLidar(&pArr, dev);

        //reset pArr size
        pArr.size = 0;
      }
    }

    void pubStatus(float* f)
    {
      //check if f is nullptr
      if(f == NULL)
      {
        ROS_WARN_STREAM("Error publishing status\n");
        return;
      }

      //publish battery
      std_msgs::Float32 batMsg;
      batMsg.data = f[0];
      socPub.publish(batMsg);

      //publish pose
      umd_arcl_driver::Pose2DStamped poseMsg;
      poseMsg.pose.x = f[1];
      poseMsg.pose.y = f[2];
      poseMsg.pose.theta = f[3];
      poseMsg.header.stamp = ros::Time::now();
      posePub.publish(poseMsg);
    }

    void pubLidar(const Point2DArr* pArr, int device)
    {
      //check if parr is nullptr
      if(pArr == NULL)
      {
        ROS_WARN_STREAM("Error publishing lidar\n");
        return;
      }

      //convert point list to pointcloud
      sensor_msgs::PointCloud msgPc;
      for(int i = 0; i < pArr->size; i++)
      {
        geometry_msgs::Point32 pt;
        pt.x = pArr->points[i].x;
        pt.y = pArr->points[i].y;
        pt.z = 0;
        msgPc.points.push_back(pt);
      }
      msgPc.header.stamp = ros::Time::now();

      //publish
      if(device == 1) lidarPub.publish(msgPc);
      else if(device == 2) lidarLowPub.publish(msgPc);
    }

    void requestStatus()
    {
      int nb = socketSend(sfd, "oneLineStatus\n");
      if(nb < 0) ROS_WARN_STREAM("Status request was unsuccessful\n");
    }

    void requestLidar()
    {
      std::string cmd = "RangeDeviceGetCurrent " + primaryLidarName + "\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Lidar data request was unsuccessful\n");
    }

    void requestLidarLow()
    {
      std::string cmd = "RangeDeviceGetCurrent " + lowLidarName + "\n";
      int nb = socketSend(sfd, cmd.c_str());
      if(nb < 0) ROS_WARN_STREAM("Low lidar data request was unsuccessful\n");
    }

    //getters
    int getSock() { return sfd; }
    int getStatusRate() { return statusRate; }
    int getLidarRate() { return lidarRate; }
};

int main(int argc, char** argv)
{
  //init ros
  ros::init(argc, argv, "arclDriver");
  ros::NodeHandle nh;
  ArclDriver Driver(&nh);

  //let ros take over
  ros::Rate rate(100);
  int statusCtr = 0;
  int lidarCtr = 0;
  while(ros::ok())
  {
    //update request counters
    statusCtr++;
    lidarCtr++;

    //send lidar request at (lidarRate) hz
    if(lidarCtr == 100/Driver.getLidarRate())
    {
      lidarCtr = 0;
      Driver.requestLidar();
      Driver.requestLidarLow();
    }

    //send status request at (statusRate) hz
    if(statusCtr == 100/Driver.getStatusRate())
    {
      statusCtr = 0;
      Driver.requestStatus();
    }

    //read line from server
    Driver.readLine();

    //process callbacks
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

