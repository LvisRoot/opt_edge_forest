/*
 * edge_forest_test.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: l_vis
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include "opencv2/core/utility.hpp"
#include "struc_forest_test/mod_structured_forest.hpp"

static const std::string SOBEL_WINDOW = "Sobel Edges window";
static const std::string FOREST_WINDOW = "Forest Edges window";

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:

	int threshold_value;
	int const max_value = 255;
	int const max_BINARY_value = 255;
	char* trackbar_value;

	cv::Ptr<cv::modXimgproc::StructuredEdgeDetection> pDollar;

	ImageConverter() :
			it_(nh_) {

		threshold_value = 0;
		trackbar_value = (char*)"Value";

		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/cam0/image_raw", 1,
				&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(SOBEL_WINDOW);
		cv::namedWindow(FOREST_WINDOW);
		cv::createTrackbar(trackbar_value, SOBEL_WINDOW, &threshold_value,
				max_value);

		pDollar =
				cv::modXimgproc::createStructuredEdgeDetection(
						"/home/l_vis/ros/catkin_ws/src/struc_forest_test/model/model.yml");
	}

	~ImageConverter() {
		cv::destroyWindow(SOBEL_WINDOW);
		cv::destroyWindow(FOREST_WINDOW);
	}

	void applySobel(cv_bridge::CvImagePtr iPtr,
			cv::Mat & edgeMask) {

		int scale = 1;
		int delta = 0;
		int ddepth = CV_16S;

		edgeMask = cv::Mat::zeros(iPtr->image.rows,iPtr->image.cols,CV_32FC3);

		cv::Mat grad_x, grad_y;
		cv::Mat abs_grad_x, abs_grad_y;
		cv::Mat sobelImg;
		cv::GaussianBlur(iPtr->image, sobelImg, cv::Size(3, 3), 0, 0,
				cv::BORDER_DEFAULT);
		//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
		cv::Sobel(sobelImg, grad_x, ddepth, 1, 0, 3, scale, delta);
		//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
		cv::Sobel(sobelImg, grad_y, ddepth, 0, 1, 3, scale, delta);
		convertScaleAbs(grad_x, abs_grad_x);
		convertScaleAbs(grad_y, abs_grad_y);
		cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobelImg);

		cv::threshold(sobelImg, sobelImg, threshold_value,
				max_BINARY_value, 0);

		for (int i = 0; i < sobelImg.rows; ++i)
			for (int j = 0; j < sobelImg.cols; ++j) {
				if (sobelImg.at<float>(i, j) < 1)
					edgeMask.at<float>(i, j)=1;
			}

		cv::imshow(SOBEL_WINDOW, sobelImg);

	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//cv::Mat edgeMask;

		//applySobel(cv_ptr, edgeMask);

		//edgeMask = cv::Mat::ones(cv_ptr->image.rows,cv_ptr->image.cols,CV_32FC3);

		//--------------------------------------------------------------------------
		//----------------------- struct forest code -------------------------------
		//--------------------------------------------------------------------------

		cv::Mat smallDst;
		//cv::Size size((int) (cv_ptr->image.cols / 2), (int) (cv_ptr->image.rows / 2));
		cv::Size size((int) (cv_ptr->image.cols ), (int) (cv_ptr->image.rows ));

		cv::resize(cv_ptr->image, smallDst, size);

		smallDst.convertTo(smallDst, cv::DataType<float>::type, 1 / 255.0);
		cv::Mat edges(smallDst.size(), smallDst.type());

		pDollar->detectEdges(smallDst, edges, threshold_value/255.0);

		// computes orientation from edge map
		//cv::Mat orientation_map;
		//pDollar->computeOrientation(edges, orientation_map);

		// suppress edges
		//cv::Mat edge_nms;
		//pDollar->edgesNms(edges, orientation_map, edge_nms, 2, 0, 1, true);

		//--------------------------------------------------------------------------

		// Update GUI Window
		cv::imshow(FOREST_WINDOW, edges);
		//cv::imshow(FOREST_WINDOW, edges);
		//cv::imshow(E_NORMAL_WINDOW, edge_nms);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
