#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "RdvObjectDetector.h"

int main(int argc, char * argv[])
{
	printf("Rendezvue Object Detector \n") ;

	if( argc != 5 )
	{
		printf("input argument error : <cmd> image_path yolo_cfg_path yolo_weights_path yolo_data_path\n") ;

		return 1 ;
	}

	std::string str_image_path = argv[1];
	std::string str_yolo_cfg_path = argv[2];
	std::string str_yolo_weight_path = argv[3];
	std::string str_yolo_data_path = argv[4];

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
	
    return EXIT_SUCCESS;
}
