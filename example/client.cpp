#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"
#include "ros/ros.h"
#include "ros_object_detector/SrvEnsemble.h"


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "ros_object_detector_client") ;
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	/*********** read test sample image path from launch file *************/
	std::string str_test_image_path_object;
	nh.getParam("test_image_path_for_object", str_test_image_path_object);	
	std::string str_image_path = 			str_test_image_path_object;
	/**********************************************************************/

	cv::Mat input_image = cv::imread(str_image_path) ; // load captured image
	
    int width = input_image.cols;
    int height = input_image.rows;
    int bpp = input_image.channels();
    int imagesize = width * height * bpp;

    uint8_t *buffer = new uint8_t[imagesize];
    memcpy(buffer, input_image.data, imagesize);
	fprintf(stderr,"[%d]\n",__LINE__);
	/****************run ros client service ******************************/
	
	fprintf(stderr,"ros Object Detector client\n") ;
	ros::ServiceClient client = n.serviceClient<ros_object_detector::SrvEnsemble>("ros_object_detector_service");
	ros_object_detector::SrvEnsemble srv;
	srv.request.str_cmd = "test";
	srv.request.width = width;
	srv.request.height = height;
	srv.request.bpp = bpp;
	fprintf(stderr,"[%d]\n",__LINE__);
	//memcpy(&srv.request.data[0], input_image.data, imagesize);
	//std::copy(input_image.data, imagesize, &srv.request.data[0]);

	for( int i = 0 ;i < imagesize ; i++ )
	{
		srv.request.data.push_back(input_image.data[i]);
	}

	fprintf(stderr,"[%d]\n",__LINE__);

	if( client.call(srv) )
	{
		fprintf(stderr,"[%d]\n",__LINE__);
	}
	else
	{
		fprintf(stderr,"[%d]\n",__LINE__);
	}
    

	ros::spin();
	return EXIT_SUCCESS;
}
