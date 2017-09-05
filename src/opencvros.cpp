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

int iLowS = 155;
int iHighS = 255;

int iLowV = 47;
int iHighV = 255;


//A typedef creates an alias which simplifies the use of multiple identifiers.
typedef union U_FloatParse
{
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;
//ReadDepthData finds out what the depth of the object is.
int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{

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

//imagecallback takes the camera sensor values, extracts the wanted HSV values from them and then finds the biggest part of the image which has this given value.
//After this a bounding box is put around that part of the image, and from the center of this box the depth value and position on the screen is calculated.
void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::CameraInfoConstPtr& right_camera)
{
    //CVBridge is used to convert the image from ROS to OpenCV format in order to use OpenCV's functions.
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
    //source opencv tutorial: Kyle Hounslaw
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

    //Remove small objects from the foreground.Morphsize (ms) can be changed to filter out small objects. 
    //The bigger the value of ms (erosion), the bigger an object has to be to get picked up by the camera.
    int ms = 25;
    int ds = 5;
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ms, ms)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ms, ms)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ds, ds)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ds, ds)) );

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

    // Print coordinates from the object
    //ROS_INFO("Center x: %d, center y: %d\n", center.x, center.y);
    drawContours(cv_ptr->image, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy); // Draw the largest contour using previously stored index.
    circle(cv_ptr->image, center, 3, Scalar(255,255,0), -1, 8, 0); // Draw dot in center of bb
    rectangle(cv_ptr->image, bounding_rect.tl(), bounding_rect.br(), Scalar(255,255,0), 2, 8, 0); // Draw bounding box

    imshow("original", cv_ptr->image); //show the original image
    camcenter = center.x;
//Make a new node 
    ros::NodeHandle nc;
    ros::Publisher ctr_pub = nc.advertise<std_msgs::Int32>("centerdata", 1);
    if (camcenter>0)
    {
        std_msgs::Int32 ctr;
        ctr.data = camcenter;
        ROS_INFO("center: %d", ctr.data);
        ctr_pub.publish(ctr);
    }
    ros::NodeHandle nd;
    ros::Publisher dpt_pub = nd.advertise<std_msgs::Int32>("depthdata", 1);
    if (camcenter>0)
    {
        std_msgs::Int32 dpt;
        dpt.data = depth;
        ROS_INFO("depth: %d", dpt.data);
        dpt_pub.publish(dpt);
    }
}
//Main function subscribes to the camera topics and enters them into the imageCallback function
int main(int argc, char** argv)
{

    ros::init(argc, argv, "opencvnode");
    ros::NodeHandle nh;

    message_filters::Subscriber<CameraInfo> camera_info_sub(nh, "/camera/depth_registered/camera_info", 1);
    message_filters::Subscriber<Image> depth_image_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<Image> rgb_image_sub(nh, "/camera/rgb/image_raw", 1);

    ros::NodeHandle nc;
    ros::Publisher ctr_pub = nc.advertise<std_msgs::Int32>("centerdata", 1);
    ros::NodeHandle nd;
    ros::Publisher dpt_pub = nd.advertise<std_msgs::Int32>("depthdata", 1);


    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), rgb_image_sub, depth_image_sub, camera_info_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    //cv::namedWindow(OPENCV_WINDOW);
    namedWindow("original", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    namedWindow("hsv", CV_WINDOW_AUTOSIZE);

    //HSV Trackbar creation, these are bound to the colored image output window.
    cvCreateTrackbar("LowH", "original", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "original", &iHighH, 179);

    cvCreateTrackbar("LowS", "original", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "original", &iHighS, 255);

    cvCreateTrackbar("LowV", "original", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "original", &iHighV, 255);

    cvStartWindowThread();


    ros::spin();
    return 0;
}
