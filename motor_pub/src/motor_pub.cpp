#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

class server {
	public:
	int echo_data;
	void EchoCallback( const std_msgs::Float32 &Echocmd);
};

void server::EchoCallback( const std_msgs::Float32 &Echocmd) {
	echo_data = Echocmd.data;
}

int main (int argc, char **argv)
{
	server objserver;
    	ros::init(argc,argv, "motor_pub");
    	ros::NodeHandle nh;
	
	cv::VideoCapture vc(0);
	if (!vc.isOpened()) return -1;

	vc.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	vc.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	cv::Mat img;

	ros::Publisher pub_dc=nh.advertise<std_msgs::Float32>("pub_dc",10);
	ros::Publisher pub_servo=nh.advertise<std_msgs::Float32>("pub_servo",10);
	ros::Subscriber sub_echo=nh.subscribe("/echo", 1, &server::EchoCallback, &objserver);

	ros::Rate loop_rate(100);

	float dc = 100;
	float servo =126; 

    	std_msgs::Float32 dc_msg;
	std_msgs::Float32 servo_msg;
	
	double ini_time = ros::Time::now().toSec();
	float error_p = 0.0, error_i = 0.0, error_d = 0.0;
	int flag_1 = 0, flag_2 = 0, flag_stop = 0;

    while(ros::ok())
    {
		int echo = objserver.echo_data;
		double time = ros::Time::now().toSec() - ini_time;
		vc >> img;
		if(img.empty()) break;
//		cv::imshow("cam",img);
		if (cv::waitKey(10)==27) {
			dc_msg.data=0;
        	servo_msg.data=90;
			pub_dc.publish(dc_msg);
        	pub_servo.publish(servo_msg);
			break;
		}
// Imgae Processing


// Processing 1. Warping
		CvMat* i = cvCreateMat(img.rows, img.cols, CV_8UC3);
		cv::Mat warp = cv::cvarrToMat(i);

		cv::Point TopLeft = cv::Point(190,175);
		cv::Point TopRight = cv::Point(390,160);
		cv::Point BottomRight = cv::Point(580,350);
		cv::Point BottomLeft = cv::Point(0,360);

		std::vector<cv::Point> rect;
		rect.push_back(TopLeft);
		rect.push_back(TopRight);
		rect.push_back(BottomRight);
		rect.push_back(BottomLeft);

		double w1 = sqrt( pow(BottomRight.x - BottomLeft.x, 2.0) + pow(BottomRight.x - BottomLeft.x, 2.0) );
		double w2 = sqrt( pow(TopRight.x - TopLeft.x, 2.0) + pow(TopRight.x - TopLeft.x, 2.0) );
		double h1 = sqrt( pow(TopRight.y - BottomLeft.y, 2.0) + pow(TopRight.y - BottomLeft.y, 2.0) );
		double h2 = sqrt( pow(TopRight.y - BottomLeft.y, 2.0) + pow(TopRight.y - BottomLeft.y, 2.0) );

		double maxWidth = (w1 < w2) ? w1 : w2;
		double maxHeight = (h1 < h2) ? h1 : h2;

		cv::Point2f src[4], dst[4];
		src[0] = cv::Point2f(TopLeft.x, TopLeft.y);
		src[1] = cv::Point2f(TopRight.x, TopRight.y);
		src[2] = cv::Point2f(BottomRight.x, BottomRight.y);
		src[3] = cv::Point2f(BottomLeft.x, BottomLeft.y);

		dst[0] = cv::Point2f(0, 0);
		dst[1] = cv::Point2f(maxWidth-1, 0);
		dst[2] = cv::Point2f(maxWidth-1, maxHeight-1);
		dst[3] = cv::Point2f(0, maxHeight-1);

		cv::Mat transformMatrix = getPerspectiveTransform(src, dst);
		warpPerspective(img, warp, transformMatrix, cv::Size(maxWidth, maxHeight));

		//imshow("Warping", warp);


// Additional Processing. Both White Lane
		Rect rect1(0, 0, warp.cols/2, warp.rows);
		Rect rect2(warp.cols/2, 0, warp.cols/2, warp.rows);
		cv::Mat roi1 = warp(rect1);
		cv::Mat roi2 = warp(rect2);

		CvMat* L = cvCreateMat(roi1.rows, roi1.cols, CV_8UC1);
		CvMat* R = cvCreateMat(roi2.rows, roi2.cols, CV_8UC1);
		cv::Mat line_L = cv::cvarrToMat(L);
		cv::Mat line_R = cv::cvarrToMat(R);

		cv::Mat rgb_L;
		cv::Mat rgb_R;
		GaussianBlur(roi1,roi1,Size(5,5),0);
		GaussianBlur(roi2,roi2,Size(5,5),0);

		cvtColor(roi1,rgb_L,CV_BGR2RGB);
		cvtColor(roi2,rgb_R,CV_BGR2RGB);

		Scalar lowerb(100,180,180);
		Scalar upperb(255,255,255);
		inRange(roi1,lowerb,upperb,line_L);
		inRange(roi2,lowerb,upperb,line_R);

		//	imshow("Left", line_L);
		//	imshow("Right", line_R);

		int flag_LW=0, flag_RW=0, cnt_LW=0, cnt_RW=0, cnt_W=0, cnt_Y=0;

		for(int j=0; j<line_L.rows; j++)
			for(int i=0; i<line_L.cols; i++)
				if(line_L.at<Vec3b>(j,i)[0]==255 && line_L.at<Vec3b>(j,i)[1]==255 && line_L.at<Vec3b>(j,i)[2]==255)
					cnt_LW++;

		for(int j=0; j<line_R.rows; j++)
			for(int i=0; i<line_R.cols; i++)
				if(line_R.at<Vec3b>(j,i)[0]==255 && line_R.at<Vec3b>(j,i)[1]==255 && line_R.at<Vec3b>(j,i)[2]==255)
					cnt_RW++;

		cnt_W = cnt_RW+cnt_LW;


		if(cnt_LW > 500)
			flag_LW = 1;

		if(cnt_RW > 500)
			flag_RW = 1;

		cv::Mat HoughL = warp;


		if(flag_LW==1 && flag_RW==1) {

			flag_1 = 1;
			flag_2 = 0;


			// Canny Edge Detection
			CvMat* l = cvCreateMat(roi1.rows, roi1.cols, CV_8UC1);
			CvMat* r = cvCreateMat(roi2.rows, roi2.cols, CV_8UC1);
			cv::Mat canny_L = cv::cvarrToMat(l);
			cv::Mat canny_R = cv::cvarrToMat(r);

			int Low_thres = 60;
			int High_thres = 110;
			Canny(line_L,canny_L,Low_thres,High_thres);
			Canny(line_R,canny_R,Low_thres,High_thres);

			// HoughLines
			std::vector<Vec2f> lines_L;
			std::vector<Vec2f> lines_R;
			HoughLines(canny_L, lines_L, 1, CV_PI / 180, 60);
			HoughLines(canny_R, lines_R, 1, CV_PI / 180, 70);
			
			for (size_t i = 0; i < lines_L.size(); i++)
			{
				float rho = lines_L[i][0], theta = lines_L[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				pt1.x = cvRound(x0 + 1000 * (-b));
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b));
				pt2.y = cvRound(y0 - 1000 * (a));
				line(HoughL, pt1, pt2, Scalar(0,0,255), 3);
			}

			for (size_t i = 0; i < lines_R.size(); i++)
			{
				float rho = lines_R[i][0], theta = lines_R[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				pt1.x = cvRound(x0 + 1000 * (-b)) + warp.cols/2;
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b)) + warp.cols/2;
				pt2.y = cvRound(y0 - 1000 * (a));
				line(HoughL, pt1, pt2, Scalar(255,0,255), 3);
			}

			imshow("lane detect", HoughL);
		}


		else {

			flag_1 = 0;
			flag_2 = 1;


		// Processing 2. Line
			// Yellow, White Region Detect
			CvMat* a = cvCreateMat(warp.rows, warp.cols, CV_8UC1);
			CvMat* b = cvCreateMat(warp.rows, warp.cols, CV_8UC1);
			cv::Mat line_Y = cv::cvarrToMat(a);
			cv::Mat line_W = cv::cvarrToMat(b);

			cv::Mat hsv;
			cv::Mat rgb;

			GaussianBlur(warp,warp,Size(5,5),0);

			cvtColor(warp,hsv,CV_BGR2HSV);
			cvtColor(warp,rgb,CV_BGR2RGB);


                        Scalar lowerb_Y(10,200,120);
			Scalar upperb_Y(50,255,230);
			inRange(hsv,lowerb_Y,upperb_Y,line_Y);

			for(int j=0; j<line_Y.rows; j++)
				for(int i=0; i<line_Y.cols; i++)
					if(line_Y.at<Vec3b>(j,i)[0]==255 && line_Y.at<Vec3b>(j,i)[1]==255 && line_Y.at<Vec3b>(j,i)[2]==255)
									cnt_Y++;


                        Scalar lowerb_W(180,180,180);
			Scalar upperb_W(255,255,255);
			inRange(rgb,lowerb_W,upperb_W,line_W);

//			imshow("Yellow", line_Y);
//			imshow("White", line_W);

			// Canny Edge Detection
			CvMat* c = cvCreateMat(warp.rows, warp.cols, CV_8UC1);
			CvMat* d = cvCreateMat(warp.rows, warp.cols, CV_8UC1);
			cv::Mat canny_Y = cv::cvarrToMat(c);
			cv::Mat canny_W = cv::cvarrToMat(d);

			int Low_th = 60;
			int High_th = 110;
			Canny(line_Y,canny_Y,Low_th,High_th);
			Canny(line_W,canny_W,Low_th,High_th);

//			imshow("Yellow Canny", canny_Y);
//			imshow("White Canny", canny_W);



			// HoughLines
			cv::Mat HoughL = warp;
			std::vector<Vec2f> lines_Y;
			std::vector<Vec2f> lines_W;

			if (cnt_W > cnt_Y) {
				HoughLines(canny_Y, lines_Y, 1, CV_PI / 180, 50, 0, 0, 0, CV_PI*2/3);
				HoughLines(canny_W, lines_W, 1, CV_PI / 180, 40);
			} else {
				HoughLines(canny_Y, lines_Y, 1, CV_PI / 180, 50);
				HoughLines(canny_W, lines_W, 1, CV_PI / 180, 60, 0, 0, CV_PI/3, CV_PI);

			}

			
			for (size_t i = 0; i < lines_Y.size(); i++)
			{
				float rho = lines_Y[i][0], theta = lines_Y[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				pt1.x = cvRound(x0 + 1000 * (-b));
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b));
				pt2.y = cvRound(y0 - 1000 * (a));
				line(HoughL, pt1, pt2, Scalar(0,0,255), 3);
			}

			for (size_t i = 0; i < lines_W.size(); i++)
			{
				float rho = lines_W[i][0], theta = lines_W[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				pt1.x = cvRound(x0 + 1000 * (-b));
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b));
				pt2.y = cvRound(y0 - 1000 * (a));
				line(HoughL, pt1, pt2, Scalar(255,0,255), 3);
			}

			imshow("lane detect", HoughL);
		}



		// Weight Calculate
		int x1=0, x2=0, mx=0, weight=0;
		int cnt1=0, cnt2=0, xRx=0, xRy=-1, xLx=0, xLy=-1;
		int x_std=0, y_std=0;
		for(int j=HoughL.rows-1; j>-1; j--) {
			for(int i=HoughL.cols-1; i>0; i--) {               
				if(HoughL.at<Vec3b>(j,i)[0]==0 && HoughL.at<Vec3b>(j,i)[1]==0 && HoughL.at<Vec3b>(j,i)[2]==255) {
					x1 = i-154;

					if (x1 > 0)
						x1 = -x1;
					else 
						x1 = x1;

					if(cnt1==0) {
						xLx = x1;
						xLy = j;
						cnt1 = 1;
					}
					break;
				}
			}

			for(int i=0; i<HoughL.cols; i++) {
				if(HoughL.at<Vec3b>(j,i)[0]==255 && HoughL.at<Vec3b>(j,i)[1]==0 && HoughL.at<Vec3b>(j,i)[2]==255) {
					x2 = i-154;

					if (x2 < 0)
                        x2 = -x2; 
					else 
						x2 = x2;

					if(cnt2==0) {
						xRx = x2;
						xRy = j;
						cnt2 = 1;
					}
					break;
				}
			}

			mx = (x1+x2)/2;
			weight += mx;
		}

		if( cnt1==0 ){
			xLx = xRx;
			xLy = xRy;
		} else if ( cnt2 == 0 ){
			xRx = xLx;
			xRy = xLy;
		}

		x_std = (xLx+xRx)/2;
		y_std = (xLy+xRy)/2;

		weight /= y_std;


		// Car Command	
	    servo = 126;

		// PID CONTROLL
        float Kp = 0.7, Ki = 0, Kd = 0.01;
		float e = atan((float(weight - x_std) / float(y_std - HoughL.rows/2)))*57.29;
		// 180/pi=57.29

		error_d = (e - error_p)/0.1;
		error_p = e;
		error_i += e;

		float PID = Kp * error_p + Ki * error_i + Kd * error_d;

		servo = 125 + int(PID);
		
        // + - changed it's pid have + - 


		dc = 100;




// Obstacle Reaction CONTROLL
		if( flag_stop == 2 ) {
			sleep(2);
			flag_stop = 0;
			servo = 96;
		}
		if( flag_stop == 1 ) {
			dc = 70;
			servo = 128;
			flag_stop = 2;
		}

		ros::spinOnce();

		float echo_Kp = 0.8; // echo P control		
			
		float echo_P = echo_Kp * echo; // error = echo
			
		if(echo < 100 && echo>=70) {
			dc = 20 + echo_P;
		} else if(echo < 70) {
			if(flag_1 == 1 && flag_2 == 0) {
				dc = 115;
				servo = 101;
			} else if(flag_1 == 0 && flag_2 == 1) {
				dc = 115;
				servo = 151;
			} else {
				dc = 50;
				servo = 128;
			}
			flag_stop = 1;
		}



		printf("weight: %d, error: %f, servo: %f \n", weight, e, servo);

		printf("-----------------------------------------------------------\n");
		printf("Distance : %d \n", echo);
		printf("echo_P : %f, dc : %f \n", echo_P, dc);
			


		ROS_INFO("TIME : %lf", time);
		if (dc<0) dc = -dc;
		else if (dc>255) dc = 255;

		if (servo<80) servo = 85;
		else if (servo>180) servo = 175;
           	else if (servo>138) dc = 110;
		else if (servo<116) dc = 110;

		printf("weight: %d, error: %f, servo: %f, dc: %f \n", weight, e, servo, dc);

        dc_msg.data=dc;
        servo_msg.data=servo;
        pub_dc.publish(dc_msg);
        pub_servo.publish(servo_msg);

		loop_rate.sleep();
    }
	cv::destroyAllWindows();
	return 0;
}
