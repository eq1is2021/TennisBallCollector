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
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;
using namespace cv;


int Bmin = 0,Bmax = 255,Gmin = 0,Gmax = 255,Rmin = 0,Rmax = 255;
int Hmin = 90,Hmax = 110,Smin = 0,Smax = 150,Vmin = 20,Vmax = 100;

void Erosion( Mat Min, Mat Mout, int erosion_type, int erosion_size, int depth)
{
    Mat element = getStructuringElement(erosion_type,
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
    : Node("node_detection_joueurs")
    {
    	RCLCPP_INFO(this->get_logger(), "Node détection des joueurs : Début du node de Détection des joueurs");
    	publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("joueurs_coords", 10);
      	subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      	"/zenith_camera/image_raw", rclcpp::SensorDataQoS(), std::bind(&DetectionBalles::image_callback, this, _1));
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
		std_msgs::msg::Header msg_header = msg->header;
		std::string frame_id = msg_header.frame_id.c_str();
		//RCLCPP_INFO(this->get_logger(), "New Image from %s", frame_id);

		Mat intrinsic = (Mat_<double>(3,3) << 325.7395873530012, 0, 640.5, 0, 325.7395873530012, 360.5, 0, 0, 1);
		Mat distCoeffs = (Mat_<double>(5,1) << 0, 0, 0, 0, 0);

		geometry_msgs::msg::PoseArray posearray;
		posearray.header.frame_id = "root_link";

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			Mat I_distorded = cv_ptr->image;
			Mat I;
	        undistort(I_distorded, I, intrinsic, distCoeffs);
			//imshow("entré", I);
			// --------------------------
			// Isole le jaune dans l'image
			// --------------------------
			Mat mask,res,hsv;
			cvtColor(I,hsv,COLOR_BGR2HSV);
			Scalar min = Scalar(Hmin,Smin,Vmin);
			Scalar max = Scalar(Hmax,Smax,Vmax);
			inRange(hsv, min, max, mask);
			bitwise_and(hsv,hsv,res,mask);
			cvtColor(res,res,COLOR_HSV2BGR);

			// --------------------------
			// Lissage de l'image
			// --------------------------
			Erosion(mask, mask, MORPH_CROSS, 2, 1);
			//Erosion(res, res, MORPH_RECT, 10, 2);

			Dilation(mask, mask, MORPH_RECT, 3, 2);
			//imshow("res",mask);


			std::vector<std::vector<Point> > contours;
			std::vector<Vec4i> hierarchy;
			findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
			std::vector<Rect> boundRect;
			for( size_t i = 0; i < contours.size(); i++){

		       Rect rect = boundingRect(contours[i]);
		       if(rect.width > 10 && rect.height > 10){
		       		boundRect.push_back(rect);
		       }
		
			}

			//on renvoie toujours un tableau de taille 2
			for (int i = 0; i < boundRect.size() and i < 2; i++){
		        rectangle( I, boundRect[i].tl(), boundRect[i].br(), Scalar(10,123,255), 2 );

		        float coords_x = 0.;
		        float coords_y = 0.;

		        float x_offset = 30.;
        		float y_offset = 683.;
        		double scale = 0.02481389578163772;

				geometry_msgs::msg::Pose p;
				if(boundRect[i].x < mask.cols/2.){
					//on est à gauche
					coords_x = boundRect[i].x + boundRect[i].width;
				}else{
					coords_x = boundRect[i].x;
				}
				coords_y = boundRect[i].y + boundRect[i].height/2;

			  	circle(I, Point(coords_x, coords_y), 8, Scalar(0, 0, 255), FILLED, LINE_8 );

				p.position.x = (coords_x-x_offset)*scale;
				p.position.y = -(coords_y-y_offset)*scale;
				p.position.z = 1;
				
				posearray.poses.push_back(p);
				
			}
			while(posearray.poses.size() < 2){
				geometry_msgs::msg::Pose p;
				p.position.x = 0;
				p.position.y = 0;
				p.position.z = 0;;
				posearray.poses.push_back(p);
			}

			//imshow("Detection Joueur",I);
			publisher_->publish(posearray);
			waitKey(3);

		}
		catch (cv_bridge::Exception& e)
		{
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
    	}
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){

  	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<DetectionBalles>());
  	rclcpp::shutdown();
  return 0;
}
