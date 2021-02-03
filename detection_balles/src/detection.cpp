#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;
using namespace cv;


int Bmin = 0,Bmax = 255,Gmin = 0,Gmax = 255,Rmin = 0,Rmax = 255;
int Hmin = 20,Hmax = 35,Smin = 165,Smax = 255,Vmin = 0,Vmax = 255;

void Erosion( Mat Min, Mat Mout, int erosion_type, int erosion_size, int depth)
{
    Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
    /// Apply the dilation operation
    for (int i = 0; i < depth; i++){
        erode( Min, Mout, element );
    }
}

void Dilation( Mat Min, Mat Mout, int dilation_type, int dilation_size, int depth)
{
    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    for (int i = 0; i < depth; i++){
        dilate( Min, Mout, element);
    }
}

class DetectionBalles : public rclcpp::Node
{
  public:
    DetectionBalles()
    : Node("node_detection")
    {
    	publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("balles_coords", 10);
      	subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      	"/zenith_camera/image_raw", 10, std::bind(&DetectionBalles::image_callback, this, _1));
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
		std_msgs::msg::Header msg_header = msg->header;
		std::string frame_id = msg_header.frame_id.c_str();
		//RCLCPP_INFO(this->get_logger(), "New Image from %s", frame_id);

		geometry_msgs::msg::PoseArray posearray;
		posearray.header.frame_id = "root_link";

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			Mat I = cv_ptr->image;
			// --------------------------
			// Isole le jaune dans l'image
			// --------------------------
			Mat mask,res,hsv;
			cvtColor(I,hsv,COLOR_BGR2HSV);
			Scalar min = Scalar(Hmin,Smin,Vmin);
			Scalar max = Scalar(Hmax,Smax,Vmax);
			inRange(hsv, min, max,mask);
			bitwise_and(hsv,hsv,res,mask);
			cvtColor(res,res,COLOR_HSV2BGR);
			//imshow("res",res);
			// --------------------------
			// Lissage de l'image
			// --------------------------
			//Erosion(res, res, MORPH_RECT, 1, 2);
			Dilation(res, res, MORPH_RECT, 1, 1);
			// --------------------------
			// Elimination du bruit
			// --------------------------
			Mat res_gray = res.clone();
			cvtColor(res, res, CV_BGR2GRAY );
			Mat labelImage, stats,centroids;
			int connectivity = 8;
			int nLabels = connectedComponentsWithStats(res, labelImage, stats, centroids, connectivity, CV_32S);
			Mat mask2(labelImage.size(), CV_8UC1, Scalar(0));
			Mat surfSup=stats.col(4)>20;

			for (int i = 1; i < nLabels; i++){
			    if (surfSup.at<uchar>(i, 0)) {
			        mask2 = mask2 | (labelImage == i);
			    }
			}
			Mat res_filtered(res.size(), CV_8UC1, Scalar(0));
			res.copyTo(res_filtered,mask2);
			// --------------------------
			// Deuxième lissage de la balle pour retrouver au maximum la rondeur.
			// --------------------------
			//imshow("res2",res_filtered);
			Mat res_dilated = res_filtered.clone();
			//Dilation(res_dilated, res_dilated, MORPH_ELLIPSE, 3, 1);
			//Erosion(res_dilated, res_dilated, MORPH_ELLIPSE, 1, 1);
			//imshow("Intermédiaire",res_dilated);
			// --------------------------
			// Isolement de la balle : Critère de circularité && Calcul cx, cy
			// --------------------------
			std::vector<std::vector<Point> > contours;
			std::vector<Vec4i> hierarchy;
			findContours( res_dilated, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
			std::vector<std::vector<Point> > contours_poly( contours.size() );
			std::vector<Rect> boundRect( contours.size() );
			std::vector<Point2f>centers( contours.size() );
			std::vector<float>radius( contours.size() );
			for( size_t i = 0; i < contours.size(); i++ )
			{
			    float perimeter = arcLength(contours[i],true);
			    float area = contourArea(contours[i],true);
			    //std::cout << 4 * M_PI * abs(area) / pow(perimeter, 2) << " " << abs(area) << std::endl;
			    if ((4 * M_PI * abs(area) / pow(perimeter, 2) > 0.6) && (abs(area) > 20)){
			        approxPolyDP( contours[i], contours_poly[i], 3, true );
			        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
			    }
			}

			for (int i = 0; i < centers.size(); i++){
				//circle( I, centers[i], (int)radius[i], 0, 2 );
				geometry_msgs::msg::Pose p;
				p.position.x = centers[i].x;
				p.position.y = centers[i].y;
				p.position.z = 0.;
				posearray.poses.push_back(p);
			}
			//imshow("Entrée",I);
			publisher_->publish(posearray);
			waitKey(3);

		}
		catch (cv_bridge::Exception& e)
		{
			RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
			return;
    	}
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionBalles>());
  rclcpp::shutdown();
  return 0;
}
