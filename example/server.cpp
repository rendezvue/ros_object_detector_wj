#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"
#include "ros/ros.h"
#include "ros_object_detector/SrvEnsemble.h"

CRdvObjectDetector *g_cls_ros_object_detector;

bool run_service(ros_object_detector::SrvEnsemble::Request &req, ros_object_detector::SrvEnsemble::Response &res) 
{	
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;
	cv::Mat input_image; // made by req;

	std::vector<Object2D> find_objects = g_cls_ros_object_detector->Run(input_image, 1) ; // run detection

	//result
	cv::Mat cmat ;
	input_image.copyTo(cmat) ;
	const int size_find_objects = find_objects.size() ;

	for( int i=0 ; i<size_find_objects ; i++ )
	{
		cv::Rect rect = cv::Rect(find_objects[i].x, find_objects[i].y, find_objects[i].w, find_objects[i].h) ;

		cv::rectangle(cmat, rect, cv::Scalar(0,0,255));
	}

	cv::imwrite("result.png", cmat) ;
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "ros_object_detector") ;
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
	printf("Rendezvue Object Detector \n") ;

	/*********** read yolov4 parameters begin*************/
	std::string str_test_image_path_object;
	nh.getParam("test_image_path_for_object", str_test_image_path_object);	

	std::string str_dnn_config_path_for_object, str_dnn_weight_path_for_object, str_dnn_meta_path_for_object;
	
	nh.getParam("dnn_config_path_for_object", str_dnn_config_path_for_object);
	nh.getParam("dnn_weight_path_for_object", str_dnn_weight_path_for_object);
	nh.getParam("dnn_meta_path_for_object", str_dnn_meta_path_for_object);	
	
	std::string str_image_path = 			str_test_image_path_object;
	std::string str_yolo_cfg_path = 		str_dnn_config_path_for_object;
	std::string str_yolo_weight_path =		str_dnn_weight_path_for_object;
	std::string str_yolo_data_path = 		str_dnn_meta_path_for_object;
	/***********read yolov4 parameters end ***************/

	cv::Mat input_image = cv::imread(str_image_path) ; // load captured image
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;
	CRdvObjectDetector cls_ros_object_detector(0, str_yolo_cfg_path, str_yolo_weight_path, str_yolo_data_path ,0.5) ; // Init yolo(darknet) + load yolov4 params

	g_cls_ros_object_detector = &cls_ros_object_detector;
	//do_service();
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;
	ros::ServiceServer service_server = n.advertiseService("ros_object_detector_service", run_service) ;
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;	
	ros::spin() ;
	fprintf(stderr,"[%d]=================== \n",__LINE__) ;
    return EXIT_SUCCESS;
}
