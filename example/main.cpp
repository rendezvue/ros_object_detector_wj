#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"
#include "ros/ros.h"
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rdv_object_detector") ;
	ros::NodeHandle nh("~");
	printf("Rendezvue Object Detector \n") ;

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

	cv::Mat input_image = cv::imread(str_image_path) ;
	CRdvObjectDetector cls_rdv_object_detector(0, str_yolo_cfg_path, str_yolo_weight_path, str_yolo_data_path ,0.5) ;
	std::vector<Object2D> find_objects = cls_rdv_object_detector.Run(input_image, 1) ;


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
//	fprintf(stderr,"[%d]=================== \n",__LINE__) ;

    return EXIT_SUCCESS;
}
