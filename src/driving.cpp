#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <image_transport/subscriber_filter.h>
#include "std_msgs/String.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

//Declaring variables used
static int drivestate = 1;
static int32_t centercv;
static int32_t depthObject;

//prints center of object and stores the value in centercv. Center of the camera is 320
void centerCallback(const std_msgs::Int32 msg)
{
    centercv = msg.data;
    ROS_INFO("center: %d", centercv);

}
//prints depth of object and stores the value in depthObject. 
void depthCallback(const std_msgs::Int32 msg)
{
    depthObject = msg.data;
    ROS_INFO("depth: %d", depthObject);

}
//drive function which uses the center and depth of the object to drive to it
void drive()
{
     ROS_INFO("FUNCTION: %d", depthObject);
    ros::NodeHandle nt;
    ros::Publisher pub = nt.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
    geometry_msgs::Twist msg;
    ros::Rate rate(10);
    //Robot drives in different states. First state is driving to the object.
    //Second state is driving the way out, waiting, and then driving back.
    //Third state is changing the color (HSV values) the robot is seeking.

    //This decides whether the robot may enter state 2. The sleeps are in place because the depth sensor scanner isn't very accurate and returns false positives.
    //When the robot scans a value less then 450 more than 3 times the robot will be allowed to enter state 2
    if (depthObject >100 && depthObject < 450)
    {   sleep(0.1);
        if (depthObject >100 && depthObject < 450)
    {sleep(0.1);
        if (depthObject >100 && depthObject < 450)
    {
        drivestate = 2;
    }
    }
    }
    if (drivestate == 1)
    {
        ROS_INFO("State 1 searching");
        if (centercv <300)
        {
            //turn left
            msg.linear.x = 0;
            msg.angular.z =0.5;
            ROS_INFO("Turning Left, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            pub.publish(msg);
        }
        if (centercv > 340)
        {
            //turn right
            msg.linear.x = 0;
            msg.angular.z = -0.5;
            ROS_INFO("Turning Right, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            pub.publish(msg);
        }
        if (centercv >300 && (centercv <340))
        {
            //go forwards
            msg.linear.x = 0.07;
            msg.angular.z = 0;
            ROS_INFO("Go Forward, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            pub.publish(msg);
        }

    }
    //Because the robot loses vision when getting too close to the object the depth scan doesn't always work well.
    //When the robot is within this distance the scanning turns off and the robot drives the necessary distance to position isself in front of the object.
    //After this is drives backwards to get ready to seek the next object.
    if (drivestate == 2 && depthObject < 490)
    {
        int a = 0;
        int b = 0;
        int c = 0;
        while (a <30)
        {
            msg.linear.x = 0.05;
            msg.angular.z = 0;
            pub.publish(msg);
            ROS_INFO("Drivestate 2, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            a++;
            rate.sleep();
        }
        while (b <30)
        {
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            ROS_INFO("Drivestate 2, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            ROS_INFO("Paused");
            b++;
            rate.sleep();
        }
        while (c <30)
        {
            msg.linear.x = -0.13;
            msg.angular.z = 0;
            pub.publish(msg);
            ROS_INFO("Drivestate 2, Center.x: %d\n", centercv);
            ROS_INFO("Depth: %d\n", depthObject);
            ROS_INFO("Backing Out");
            c++;
            rate.sleep();
        }
        drivestate++;
    }
//In drivestate 3 the speed values are reset and the seeked HSV values are changed.
    if (drivestate == 3)
    {
        ROS_INFO("Drivestate 3, Center.x: %d\n", centercv);
        msg.linear.x = 0;
        msg.angular.z = 0;

        //Rood Cola
        int iLowH = 0;
        int iHighH = 29;

        int iLowS = 211;
        int iHighS = 255;

        int iLowV = 11;
        int iHighV = 190;

        rate.sleep();
        drivestate = 1;
    }

}

//Main function, uses 
int main(int argc, char **argv)
{

    ros::init(argc, argv, "drivingnode");

    ros::NodeHandle n;
//Center en Depth subscriber
    ros::Subscriber subc = n.subscribe("centerdata", 1000, centerCallback);
    ros::Subscriber subd = n.subscribe("depthdata", 1000, depthCallback);
    ros::NodeHandle nt;
    geometry_msgs::Twist msg;
    ros::Publisher pub = nt.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    drive();


    ros::spin();

    return 0;
}

