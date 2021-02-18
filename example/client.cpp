#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"
#include "ros/ros.h"
#include "ros_object_detector/SrvEnsemble.h"


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "ros_object_detector_client") ;
	ros::NodeHandle n;
	printf("ros Object Detector client\n") ;

	ros::ServiceClient client = n.serviceClient<ros_object_detector::SrvEnsemble>("ros_object_detector_service");

	ros_object_detector::SrvEnsemble srv;

	srv.request.str_cmd = "test";

	if( client.call(srv) )
	{
		printf("[%d]\n",__LINE__);
	}
	else
	{
		printf("[%d]\n",__LINE__);
	}
    return EXIT_SUCCESS;
}
