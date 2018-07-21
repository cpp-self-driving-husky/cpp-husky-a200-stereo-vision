#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;
using namespace ros;

//Set constants
#define LINEAR .4	//The linear speed for the Husky (slow speed when using the controller)
#define ANGULAR .6	//The angular speed for the Husky (slow speed when using the controller)
#define DIS_THRESH 1	//Distance from Husky to avoid an obstacle 5 seconds away
#define PER_THRESH 15	//Percentage of the section that must be within the DIST_THRESH for it to be seen as occupied
#define IMAGE_TOPIC "/camera/disparity"	//The topic to subscribe to for the depth image
#define MOVEMENT_TOPIC "/cmd_vel"	//The topic to publish move commands to (In this case it publishes a string with the direction)
#define GPS_TOPIC ""	//The topic to subscribe to for the gps information
#define OPENCV_WINDOW "Image window"	//Name of the OpenCV window that will show the image
#define PI 3.14159265358979323	//The value of PI if needed for math later

//Global Variables
int width = 0;	//The overall width of the image
int height = 0;	//The overall height of the image
int roi_w = 0;	//Width of the ROI
int roi_h = 0;	//Height of the ROI
int x_start = 0;	//The top-left pixel of the ROI
int x_end = 0;		//The bottom-right pixel of the ROI
int y_start = 0;	//The top-left pixel of the ROI
int y_end = 0;		//The bottom-right pixel of the ROI
float f = 0;	//The focal length of the camera used to calculate depth
float T = 0;	//The baseline length of the camera used to calculate depth
int section = 0; 	//The section that is selected

class ImageConverter
{
	NodeHandle nodeHandle;
	Subscriber sub;
	Publisher pub;

public:
	ImageConverter()
	{
		// Subscribe to input video feed and publish output direction feed
		sub = nodeHandle.subscribe(IMAGE_TOPIC, 1, &ImageConverter::processImage, this);
		pub = nodeHandle.advertise<geometry_msgs::Twist>(MOVEMENT_TOPIC, 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

  	//Sets the depthImage to the image from the subscribed topic
	//void processImage(const sensor_msgs::ImageConstPtr& msg)
	void processImage(const stereo_msgs::DisparityImage& msg)
	{
		sensor_msgs::Image img = msg.image;
		cv_bridge::CvImagePtr cv_ptr;	//Used to hold OpenCV image
		cv::Mat depthImage;

		//Convert depth image to OpenCV image
		try
		{
			//cv_ptr = cv_bridge::toCvCopy(msg);	//Convert image if possible
			cv_ptr = cv_bridge::toCvCopy(img);	//Convert image if possible
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		depthImage = cv_ptr->image;	//Set the depthImage to the image of the cv_ptr

		//Set the camera information
		//width = img.width;
		//height = img.height;
		width = depthImage.cols;
		height = depthImage.rows;
		x_start = 0;
		y_start = 0;
		x_end = width;
		y_end = height;
		calculateCorners(depthImage);
		roi_w = x_end - x_start;
		roi_h = y_end - y_start;
		f = msg.f;
		T = msg.T;
		cout << "ROI Width: " << roi_w << "   ROI Height: " << roi_h << endl;
		cout << "X_Start: " << x_start << "   Y_Start: " << y_start << endl;
		cout << "X_end: " << x_end << "   Y_End: " << y_end << endl;

		//Create the partition lines (@ 1/3, @ 1/2, and @ 2/3 of image)
		int partitionLines[3];
		for(int i = 0; i < 3; i++)
			partitionLines[i] = 0;
		partition(partitionLines, depthImage);
		cout << "Left: " << partitionLines[0] << "   Center: " << partitionLines[1] << "   Right: " << partitionLines[2] << endl;

		if(depthImage.cols > 0 && depthImage.rows > 0)
		{
			//Get the depth at the center of the image
			//float depth = depthImage.at<float>(depthImage.cols / 2, depthImage.rows / 2);
			//float disparity = depthImage.at<float>(depthImage.cols / 2, depthImage.rows / 2);
			//float f = msg.f;
			//float T = msg.T;
			//float depth = (f * T) / disparity;
			//cout << "Depth: " << depth << endl;

			//Normalize the image so it is able to be printed neatly
			double max = 0.0;
			cv::minMaxLoc(depthImage, 0, &max, 0, 0);
			cv::Mat normalized;
			depthImage.convertTo(normalized, CV_32F, 1.0/max, 0);

			// Draw a circle on the video stream in the center
			//cv::circle(normalized, cv::Point(x_start, y_start), 10, cv::Scalar(255, 255, 255), 5);
			cv::circle(normalized, cv::Point(x_end, y_end), 10, cv::Scalar(255, 255, 255), 5);
			cv::line(normalized, cv::Point(partitionLines[0], y_start), cv::Point(partitionLines[0], y_end), cv::Scalar(255, 255, 255), 3);
			cv::line(normalized, cv::Point(partitionLines[1], y_start), cv::Point(partitionLines[1], y_end), cv::Scalar(255, 255, 255), 3);
			cv::line(normalized, cv::Point(partitionLines[2], y_start), cv::Point(partitionLines[2], y_end), cv::Scalar(255, 255, 255), 3);

			// Update GUI Window
			//cv::imshow(OPENCV_WINDOW, normalized);
			//cv::waitKey(3);
		}

		/*
			sectionValues holds the percent of pixels wihtin the given threshold
			Pos 0: The left third of the image
			Pos 1: The center thrid of the image
			Pos 2: The right third of the image
			Pos 3: The left half of the imgae
			Pos 4: The right half of the image
		*/
		float sectionValues[5];
		for(int i = 0; i < 5; i++)
			sectionValues[i] = 0;
		//Count pixels and increase appropriate counters (3 sections + left/right sides = 5 counters)
		countPixels(depthImage, sectionValues, partitionLines);

		//Select section to move to
		section = selectSection(sectionValues);
		cout << "Section: " << section << "\n\n";

		//Send movement commands (More detail on this later)
		//std_msgs::String direction;
		geometry_msgs::Twist direction;
		switch(section)
		{
			case 0:
				direction.linear.x = LINEAR;
				direction.angular.z = ANGULAR * -1;
				break;
			case 1:
				direction.linear.x = LINEAR;
				direction.angular.z = 0;
				break;
			case 2:
				direction.linear.x = LINEAR;
				direction.angular.z = ANGULAR;
				break;
			default:
				direction.linear.x = 0;
				direction.angular.z = 0;
				break;
		}
		pub.publish(direction);
	}

	//Calculate the top-left and bottom-right corners of the ROI
	void calculateCorners(const cv::Mat& depthImage)
	{
		bool empty = true;
		//Find the fisrt row that is not 0 all the way across
		while(empty)
		{
			for(int x = 0; x < width; x++)
			{
				//Break out if the spot is not 0, otherwise move down one row
				if(depthImage.at<float>(y_start, x) != 0)
				{
					empty = false;
					break;
				}
			}
			if(empty)
				y_start++;
		}

		empty = true;
		//Find the first column that is not 0 all the way across
		while(empty)
		{
			for(int y = y_start; y < height; y++)
			{
				//Break out if the spot is not 0, otherwise move down one row
				if(depthImage.at<float>(y, x_start) != 0)
				{
					empty = false;
					break;
				}
			}
			if(empty)
				x_start++;
		}

		empty = true;
		//Find the last row that is not 0 all the way across
		while(empty)
		{
			for(int x = x_end; x > x_start; x--)
			{
				//Break out if the spot is not 0, otherwise move down one row
				if(depthImage.at<float>(y_end, x) != 0)
				{
					empty = false;
					break;
				}
			}
			if(empty)
				y_end--;
		}

		empty = true;
		//Find the last column that is not 0 all the way across
		while(empty)
		{
			for(int y = y_end; y > y_start; y--)
			{
				//Break out if the spot is not 0, otherwise move down one row
				if(depthImage.at<float>(y, x_end) != 0)
				{
					empty = false;
					break;
				}
			}
			if(empty)
				x_end--;
		}
	}

	void partition(int *partitionLines, const cv::Mat& image)
	{
		//Includes the x_offset so we get the position of the lines
		partitionLines[0] = roi_w / 3 + x_start;	//Seperates left and center sections (@ 1/3 of ROI)
		partitionLines[1] = roi_w / 2 + x_start;	//Seperates right and left of the image (@ 1/2 of ROI)
		partitionLines[2] = roi_w * (2.0 / 3) + x_start;	//Seperates center and right sections (@ 2/3 of ROI)
	}

	void countPixels(const cv::Mat& image, float *sectionValues, int *partitionLines)
	{
		int counters[5];	//Holds number of pixels closer than threshold in each section
		int totals[5];		//Holds total number of pixels in each section
		for(int i = 0; i < 5; i++)
		{
			counters[i] = 0;
			totals[i] = 0;
		}

		//Move through the array to each pixel of the ROI
		for(int y = y_start; y < y_end; y++)
		{
			for(int x = x_start; x < x_end; x++)
			{
				//Get the depth at the center of the image
				//float depth = image.at<float>(x, y);
				float disparity = image.at<float>(y, x);
				float depth = (f * T) / disparity;

				//Pixel is in the left third of the image
				if(x < partitionLines[0])
				{
					totals[0]++;	//Increase left third total
					totals[3]++;	//Increase left half total
					if(depth <= DIS_THRESH || depth == NAN)
					{
						counters[0]++;	//Increase left third counter
						counters[3]++;	//Increase left half counter
					}
				}
				//Pixel is in the center of the image
				else if(x < partitionLines[2])
				{
					totals[1]++;	//Increase center third total
					//Pixel is in the left half of the image
					if(x < partitionLines[1])
						totals[3]++;	//Increase left half total
					//Pixel is in the right half of the image
					else
						totals[4]++;	//Increase right half total
					if(depth <= DIS_THRESH || depth == NAN)
					{
						counters[1]++;	//Increase center third counter
						//Pixel is in the left half of the image
						if(x < partitionLines[1])
							counters[3]++;	//Increase left half counter
						//Pixel is in the right half of the image
						else
							counters[4]++;	//Increase right half counter
					}
				}
				//Pixel is in the right third of the image
				else
				{
					totals[2]++;	//Increase right third total
					totals[4]++;	//Increase right half total
					if(depth <= DIS_THRESH || depth == NAN)
					{
						counters[2]++;	//Increase right third counter
						counters[4]++;	//Increase right half counter
					}
				}
			}
		}
	
		for(int i = 0; i < 5; i++)
			sectionValues[i] = ((float)counters[i] / totals[i]) * 100;
		cout << "Left Third: " << counters[0] << "/" << totals[0] << " = " << sectionValues[0] << endl;
		cout << "Center Third: " << counters[1] << "/" << totals[1] << " = " << sectionValues[1] << endl;
		cout << "Right Third: " << counters[2] << "/" << totals[2] << " = " << sectionValues[2] << endl;
		cout << "Left Half: " << counters[3] << "/" << totals[3] << " = " << sectionValues[3] << endl;
		cout << "Right Half: " << counters[4] << "/" << totals[4] << " = " << sectionValues[4] << endl;
	}

	int selectSection(float *sectionValues)
	{
		int section = -1;	//Holds the selected section
		//If the center section is clear, choose it
		if(sectionValues[1] < PER_THRESH)
			section = 1;
		//If the left is more clear than the right and the left is within the percent threshold choose the left
		else if(sectionValues[0] < sectionValues[2] && sectionValues[0] < PER_THRESH)
			section = 0;
		//If the right is more clear than the left and the right is within the percent threshold choose the right
		else if(sectionValues[2] < sectionValues[0] && sectionValues[2] < PER_THRESH)
			section = 2;
		//If the two sections are the same and within the percent threshold look at the left and right halves
		else if(sectionValues[0] == sectionValues[2] && sectionValues[0] < PER_THRESH)
		{
			//If the left half is more clear than the right half, choose the left half
			if(sectionValues[3] < sectionValues[4])
				section = 0;
			//Otherwise choose the right half
			else
				section = 2;
		}
		//No section is clear so don't choose any section
		else
			section = -1;
		return section;
	}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
	ImageConverter ic;
    //ros::spinOnce();
	ros::spin();
    return 0;
}















