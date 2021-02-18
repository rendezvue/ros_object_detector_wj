#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"
#include "ros/ros.h"
#include "ros_object_detector/SrvEnsemble.h"
#include "ros_object_detector/yolo.h"

bool service_call(std::string image_path)
{
	ros::NodeHandle n;
	/****** load captured image & read image info ****************/
	cv::Mat input_image = cv::imread(image_path) ; 
	
    int width = input_image.cols;
    int height = input_image.rows;
    int bpp = input_image.channels();
    int imagesize = width * height * bpp;

	/****************run ros client service ******************************/	
	fprintf(stderr,"ros Object Detector client\n") ;
	ros::ServiceClient client = n.serviceClient<ros_object_detector::SrvEnsemble>("ros_object_detector_service");
	ros_object_detector::SrvEnsemble srv;
	srv.request.str_cmd = "run";
	srv.request.width = width;
	srv.request.height = height;
	srv.request.bpp = bpp;
	//memcpy(&srv.request.data[0], input_image.data, imagesize);
	//std::copy(input_image.data, imagesize, &srv.request.data[0]);

	for( int i = 0 ;i < imagesize ; i++ ) // copy "image buffer" -> "ros msg vector"
	{
		srv.request.data.push_back(input_image.data[i]);
	}
	
	if( client.call(srv) )
	{
		fprintf(stderr,"Service call success\n");

		for( int i = 0 ;i < srv.response.result.size(); i++)
		{
			ros_object_detector::yolo ret_data = srv.response.result[i];
			
			fprintf(stderr,"x:%d, y:%d, w:%d, h:%d, score:%f\n", ret_data.x, ret_data.y, ret_data.width, ret_data.height, ret_data.score );
		}
	}
	else
	{
		fprintf(stderr,"Service Failed\n");
	}
    
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "ros_object_detector_client") ;

	ros::NodeHandle nh("~");
	/*********** read test sample image path from launch file *************/
	std::string str_test_image_path_object;
	nh.getParam("test_image_path_for_object", str_test_image_path_object);	
	std::string str_image_path = 			str_test_image_path_object;
	/**********************************************************************/

	service_call(str_image_path);

	ros::spin();
	return EXIT_SUCCESS;
}
