/* EXAMPLE 2 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Load, Modify, and Save an Image"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html#load-save-image
/*
/*   Aditional: Presentation "ROS_Lecture10.pptx"
/*   File: Annexed
/* 
/*   it was modificated by Hernán Hernández to be use like own template 
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_grayImage_hh"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "Gray Image"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Image get from camera (raw). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Image public to ROS (processed).

// CLASS: Image Conver (OpenCV)
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
	    cv::Mat cvMat_OriginalImage_ptr = cv_OriginalImage_ptr->image;

	    // Transform Original Image to Gray format.
	    cv::Mat cvMat_GrayImage_ptr;
	    cv::cvtColor(cvMat_OriginalImage_ptr, cvMat_GrayImage_ptr, CV_BGR2GRAY); 

	    /*********************************/ 
	    /* END: digital image processing */
	    /*********************************/

    	    // Update GUI Window1 - Original Image
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

	    // Update GUI Window2 - Gray Image
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_ptr);
    	    cv::waitKey(3);

    	    // Convert OpenCV image (Mat) to OpenCV image (Bridge) to ROS image (Topic) 
	    cv_bridge::CvImage cv_GrayImage; // it's needed use the Class CvImage not CvImagePtr
	    cv_GrayImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    cv_GrayImage.encoding = sensor_msgs::image_encodings::MONO8; // MONO8 AND MONO16 are equal to GRAY format
	    cv_GrayImage.image = cvMat_GrayImage_ptr; // data
    	    // Output modified video stream
	    topic1_pub__image_output.publish(cv_GrayImage.toImageMsg());
  	}
};

//---Main---
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

// Bla bla bla