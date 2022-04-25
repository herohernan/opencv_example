/* EXAMPLE 3 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Changing the contrast and brightness of an image!"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html#basic-linear-transform
/*
/*   For implement the scrollbar/trackbar was used the tutorial: "Adding a Trackbar to our applications!"
/*   From = https://docs.opencv.org/2.4/doc/tutorials/highgui/trackbar/trackbar.html
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
#define    NODE_NAME       	"opencv_change_contrast_hh"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "New Image (Contrast & brightness)"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Image get from camera (raw). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Image public to ROS (processed).

//***STATIC FUNCTION: Trackbar Method for Alpha value. High-GUI of OpenCV***
double Alpha = 1.5; // Simple contrast control. Value from 1.0 to 3.0 
int trackbar1_slider; // where is stored the actual trackbar value

static void trackbar1_func(int, void*)
{
    // scale trackbar value [0 - 100%] to alpha value [0.0 - 3.0]
    Alpha = trackbar1_slider*3.0/100.0;
}

//***STATIC FUNCTION: Trackbar Method for Beta value. High-GUI of OpenCV***
int Beta = 30;  // Simple brightness control. Value form 0 to 100	
int trackbar2_slider; // where is stored the actual trackbar value

static void trackbar2_func(int, void*)
{
    // Change Beta value
    Beta = trackbar2_slider;
}

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

	    // Create a new Scrollbar/trackbar
	    int trackbar_maxValue = 100; // In percent.
	    cv::createTrackbar("Alpha [0-100%]", OPENCV_WINDOW2, &trackbar1_slider, trackbar_maxValue, trackbar1_func); // Note the following: 1) Our Trackbar has a label "Alpha", 2) The Trackbar is located in the window “OPENCV_WINDOW2”, 3) The Trackbar values will be in the range from 0 to "trackbar_maxValue" (the minimum limit is always zero), 4) The numerical value of Trackbar is stored in "trackbar_slider", and 5) Whenever the user moves the Trackbar, the callback function on_trackbar is called
	    cv::createTrackbar("Beta [0-100%]", OPENCV_WINDOW2, &trackbar2_slider, trackbar_maxValue, trackbar2_func); 
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
	    cv::Mat cvMat_Image_ptr = cv_OriginalImage_ptr->image;

	    // Do the operation new_image(i,j) = Alpha*Orginal_image(i,j) + Beta
 	    cv::Mat cvMat_NewImage_ptr = cv::Mat::zeros(cvMat_Image_ptr.size(), cvMat_Image_ptr.type()); // Matrix of 0 of the same size
	    for( int y = 0; y < cvMat_Image_ptr.rows; y++ )
    	    { 
		for( int x = 0; x < cvMat_Image_ptr.cols; x++ )
         	{ 
		    for( int c = 0; c < 3; c++ )
              	    {
      			cvMat_NewImage_ptr.at<cv::Vec3b>(y,x)[c] = 
				cv::saturate_cast<uchar>( Alpha*(cvMat_Image_ptr.at<cv::Vec3b>(y,x)[c]) + Beta );
             	    }
    		}
    	    }
	    /*********************************/ 
	    /* END: digital image processing */
	    /*********************************/

    	    // Update GUI Window1 - Original Image
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

	    // Update GUI Window2 - New Image (Contrast & brightness)
   	    cv::imshow(OPENCV_WINDOW2, cvMat_NewImage_ptr);
	    ROS_INFO("Alpha %f ------ Beta %d", Alpha, Beta);
	    cv::waitKey(3);

    	    // Convert OpenCV image (Mat) to OpenCV image (Bridge) to ROS image (Topic)  
	    cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	    cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image 
	    cv_NewImage.image = cvMat_NewImage_ptr; // data
    	    // Output modified video stream
	    topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
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
