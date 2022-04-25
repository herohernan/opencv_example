/* EXAMPLE 5 USING OPENCV
/*
/*   Base: Hough Circle Transform
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html#hough-circle
/*
/*   Base to draw into OpenCV: 
/*   from = https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html
/*   
/*   Example in ROS:
/*   from: https://github.com/epsilonorion/ros_tutorials/blob/master/opencv_tut/src/findCircle.cpp
/* 
/*   it was modificated by Hernán Hernández to be use like own template 
*/

// Includes
#include <ros/ros.h>
#include <stdio.h> // needed to use the class: "vector"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>*/

// Defines - General
#define    NODE_NAME       	"opencv_houghCircleTransform_hh"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "Image Filtered"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Image get from camera (raw). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Image public to ROS (processed).

//***CLASS: Image Conver (OpenCV)***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Image used 
    	image_transport::ImageTransport it_; // Object it_ from image transport clase (used to the digital image processing)
    	image_transport::Subscriber topic1_sub__image_input; // Image get from camera (raw). ROS format (Topic)
    	image_transport::Publisher topic1_pub__image_output; // Image public to ROS (processed). ROS format (Topic)

    public:

	/* Constructor Method. 
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Topics declaration
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

	    // Create the GUI Windows (where print the images)
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // close the GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}

	/* associate to "TOPIC1_SUB__IMAGE_INPUT" which get  Image get from camera (raw) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg is the Image get from camera (raw)
  	{
	    // Convert ROS image (Topic) to OpenCV image (Ptr)	    
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    	    }
	    catch (cv_bridge::Exception& e)
    	    {
		// Print a error if it is detected
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

	    /****************************/ 
	    /* digital image processing */
	    /****************************/
	   
    	    // Convert data to cv::Mat class
	    cv::Mat cvMat_Image = cv_OriginalImage_ptr->image;

	    // Transform Original Image to Gray format.
	    cv::Mat cvMat_GrayImage;
	    cv::cvtColor(cvMat_Image, cvMat_GrayImage, CV_BGR2GRAY); 

	    // Reduce the noise applied a Gaussian filter, so we avoid false circle detection
	    cv::Mat cvMat_GrayImage_filtered;
	    cv::GaussianBlur(cvMat_GrayImage, cvMat_GrayImage_filtered, cv::Size(9, 9), 2, 2);
		    
	    // Apply the Hough Transform to find the circles
	    std::vector<cv::Vec3f> circles;
	    cv::HoughCircles(cvMat_GrayImage_filtered, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 155, 0, 0 );
	    /* Example: HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
	       Parameters:
	           src_gray: Input image (grayscale)
    	           circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
	     	   CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
	           dp = 1: The inverse ratio of resolution TODO
	           min_dist = src_gray.rows/8: Minimum distance between detected centers TODO
	     	   param_1 = Upper threshold for the internal Canny edge detector TODO
	           param_2 = Threshold for center detection. TODO
    	           min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
    	          max_radius = 0: Maximum radius to be detected. If unknown, put zero as default */
	  
	    // Circles detected
	    for(size_t i = 0; i < circles.size(); i++) //size_t is a variable which can store the maximum size of a theoretically possible object, in this case the length of "i" (unknown size)  
	    {
		// Draw circles over the original image
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
 		int radius = cvRound(circles[i][2]);
   		circle(cvMat_Image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 ); // circle center (with radius=3, Color green)
   		circle(cvMat_Image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 ); // circle outline (With real radius, Color red)

		/* circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		   Parameters:
		       img – Image where the circle is drawn.
		       center – Center of the circle.
    		       radius – Radius of the circle.
    		       color – Circle color.
    		       thickness – Thickness of the circle outline. If it's negative thickness means that a filled circle is to be drawn.
    		       lineType – Type of the circle boundary. See the line() description. Default 8
    		       shift – Number of fractional bits in the coordinates of the center and in the radius value. Deafult 0*/

		// Print terminal 
		ROS_INFO("Circle detected #%d / %d: ", int(i)+1, (int)circles.size());
		ROS_INFO("    x=%d, y=%d, r=%d: ", cvRound(circles[i][0]), cvRound(circles[i][1]), cvRound(circles[i][2]));
      	    }

	    /*********************************/ 
	    /* END: digital image processing */
	    /*********************************/

    	    // Update GUI Window1 - Original Image
   	    cv::imshow(OPENCV_WINDOW1, cvMat_Image);
    	    cv::waitKey(3);

	    // Update GUI Window2 - Filter applied
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_filtered);
	    cv::waitKey(3);
	}
};

//***Main***
int main(int argc, char** argv)
{
    // Init ROS 
    ros::init(argc, argv, NODE_NAME);
  
    // Init object from class ImageConverter, defined above
    ImageConverter ic;

    // While true. Getting data from subscribe Topic
    ros::spin();
    return 0;
}
