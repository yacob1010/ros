#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind/protect.hpp>
#include <boost/bind.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
int drivestate = 1;

static int camcenter;
static int funcscan = 0;
static int mainscan = 0;


/* Groen flesje
int iLowH = 30;
int iHighH = 69;

int iLowS = 152;
int iHighS = 255;

int iLowV = 41;
int iHighV = 166;
*/
/*Blauw Flesje
int iLowH = 94;
int iHighH = 134;

int iLowS = 65;
int iHighS = 180;

int iLowV = 29;
int iHighV = 127;
*/
/*Rood Cola
int iLowH = 0;
int iHighH = 29;

int iLowS = 211;
int iHighS = 255;

int iLowV = 11;
int iHighV = 190;
*/
//Groen Arizona
int iLowH = 19;
int iHighH = 98;

int iLowS = 1;
int iHighS = 211;

int iLowV = 47;
int iHighV = 158;



typedef union U_FloatParse
{
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    //credits opencv math to Kyle Hounslaw
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4)
    {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
                ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1)))   // Both lil endian
        {
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++)
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
    int temp_val;
    // If big endian
    if (depth_image->is_bigendian)
        temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
    // If little endian
    else
        temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
    // Make sure data is valid (check if NaN)
    if (temp_val == temp_val)
        return temp_val;
    return -1;  // If depth data invalid
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::CameraInfoConstPtr& right_camera)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal;
    Mat imgHSV;
    cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from RGB to HSV
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    double m01, m10, area;
    int posX = 0, posY = 0;

    Moments oMoments = moments(imgThresholded);
    m01 = oMoments.m01;
    m10 = oMoments.m10;
    area = oMoments.m00;

    // Filter position
    if (area > 1000000)
    {
        // Position
        posX = m10 / area;
        posY = m01 / area;
    }

    //morphological opening (remove small objects from the foreground)
    //morphsize to filter out small objects
    int ms = 20;
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ms, ms)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ms, ms)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

    imshow("hsv", imgThresholded); //show the thresholded image


    int largest_area = 0;
    int largest_contour_index = 0;
    double width, height;

    vector< vector<Point> > contours; // Vector for storing contour
    vector<Vec4i> hierarchy;
    Rect bounding_rect;

    // Find all contours in thresholded image
    findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Find the biggest contour and calculate bounding box
    for(int i = 0; i < contours.size(); i++)
    {
        double a = contourArea(contours[i], false);
        if(a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
            bounding_rect = boundingRect(contours[i]);
        }
    }

    // Calculate width and heigt of bounding box
    width  = bounding_rect.br().x - bounding_rect.tl().x;
    height = bounding_rect.br().y - bounding_rect.tl().y;

    // Calculate center.x
    Point center = Point(((width/2) + bounding_rect.tl().x), ((height/2) + bounding_rect.tl().y));
    int depth = ReadDepthData(center.y, center.x, msg_depth);

    // print coordinates from the object
    //ROS_INFO("Center x: %d, center y: %d\n", center.x, center.y);
    drawContours(cv_ptr->image, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy); // Draw the largest contour using previously stored index.
    circle(cv_ptr->image, center, 3, Scalar(255,255,0), -1, 8, 0); // Draw dot in center of bb
    rectangle(cv_ptr->image, bounding_rect.tl(), bounding_rect.br(), Scalar(255,255,0), 2, 8, 0); // Draw bounding box

    imshow("original", cv_ptr->image); //show the original image
    camcenter = center.x;

    /*
    ros::Rate rate(10);
    {
        geometry_msgs::Twist msg;
    //Robot drives in different states. First state is driving to the object.
    //Second state is driving the way out, waiting, and then driving back.
    //Third state is changing the color (HSV values) the robot is seeking.

        if (depth >100 && depth < 450)
        {
            drivestate = 2;
        }
        if (drivestate == 1)
        {
            if (center.x <300)
            {
                //turn left
                msg.linear.x = 0;
                msg.angular.z =0.5;
                ROS_INFO("Turning Left, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                pub.publish(msg);
            }
            if (center.x > 340)
            {
                //turn right
                msg.linear.x = 0;
                msg.angular.z = -0.5;
                ROS_INFO("Turning Right, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                pub.publish(msg);
            }
            if (center.x >300 && (center.x <340))
            {
                //go forwards
                msg.linear.x = 0.07;
                msg.angular.z = 0;
                ROS_INFO("Go Forward, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                pub.publish(msg);
            }

        }
        if (drivestate == 2 && depth < 490)
        {
            int a = 0;
            int b = 0;
            int c = 0;
            while (a <30)
            {
                msg.linear.x = 0.05;
                msg.angular.z = 0;
                pub.publish(msg);
                ROS_INFO("Drivestate 2, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                a++;
                rate.sleep();
            }
            while (b <30)
            {
                msg.linear.x = 0;
                msg.angular.z = 0;
                pub.publish(msg);
                ROS_INFO("Drivestate 2, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                ROS_INFO("Paused");
                b++;
                rate.sleep();
            }
            while (c <30)
            {
                msg.linear.x = -0.13;
                msg.angular.z = 0;
                pub.publish(msg);
                ROS_INFO("Drivestate 2, Center.x: %d\n", center.x);
                ROS_INFO("Depth: %d\n", depth);
                ROS_INFO("Backing Out");
                c++;
                rate.sleep();
            }
            drivestate++;
        }

        if (drivestate == 3)
        {
            ROS_INFO("Drivestate 3, Center.x: %d\n", center.x);
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
    }*/
 funcscan++;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "opencvnode");
    ros::NodeHandle nh;

    message_filters::Subscriber<CameraInfo> camera_info_sub(nh, "/camera/depth_registered/camera_info", 1);
    message_filters::Subscriber<Image> depth_image_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<Image> rgb_image_sub(nh, "/camera/rgb/image_raw", 1);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);

    ros::NodeHandle nt;
    ros::Publisher chatter_pub = nt.advertise<std_msgs::Int32>("camdata", 1);


    //message_filters::Subscriber<std_msgs::String> coordinates_array(nh_, "chatter" , 1);

    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), rgb_image_sub, depth_image_sub, camera_info_sub);
    //TimeSynchronizer<Image, Image, CameraInfo> sync(rgb_image_sub, depth_image_sub, camera_info_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    //cv::namedWindow(OPENCV_WINDOW);
    namedWindow("original", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    namedWindow("hsv", CV_WINDOW_AUTOSIZE);

    // TODO change to hsv values loaded in. For the green card it is: H:42, 90. S:90,255. V:0,255
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "original", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "original", &iHighH, 179);

    cvCreateTrackbar("LowS", "original", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "original", &iHighS, 255);

    cvCreateTrackbar("LowV", "original", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "original", &iHighV, 255);

    cvStartWindowThread();
if (funcscan > mainscan){

        std_msgs::Int32 msg;
        msg.data = camcenter;
        chatter_pub.publish(msg);
        ROS_INFO("test %d", msg.data);
        mainscan = funcscan;
}


    ros::spin();
    return 0;
}
