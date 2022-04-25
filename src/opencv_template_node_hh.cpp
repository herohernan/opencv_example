/* Template to use Open Cv inside ROS.
/*
/*   Base: ROS tutorial
/*   from = http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 
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
#define    NODE_NAME       	"opencv_template_node_hh"
#define    OPENCV_WINDOW       "Image window"

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

	    // Create the GUI Window (where print the image)
    	    cv::namedWindow(OPENCV_WINDOW);
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // close the GUI Window
    	    cv::destroyWindow(OPENCV_WINDOW);
  	}

	/* associate to "TOPIC1_SUB__IMAGE_INPUT" which get  Image get from camera (raw) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg is the Image get from camera (raw)
  	{
	    // Convert ROS image (Topic) to OpenCV image (Ptr)
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr; // copy the Topic image from ROS to CvImage from OpenCV
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // possible constants: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"... Option when use "cv_bridge::CvImagePtr" data type
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
    	    
	    // TODO TODO TODO

	    /*********************************/ 
	    /* END: digital image processing */
	    /*********************************/

    	    // Draw an example circle on the video stream
    	    if (cv_OriginalImage_ptr->image.rows > 60 && cv_OriginalImage_ptr->image.cols > 60)
      	    cv::circle(cv_OriginalImage_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    	    // Update GUI Window
   	    cv::imshow(OPENCV_WINDOW, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

    	    // Convert OpenCV image (Ptr) to ROS image (Topic) 
	    // Output modified video stream
    	    topic1_pub__image_output.publish(cv_OriginalImage_ptr->toImageMsg());
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
