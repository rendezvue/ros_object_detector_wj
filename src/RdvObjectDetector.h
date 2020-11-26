#ifndef INCLUDE_RDV_OBJECT_DETECTOR_H
#define INCLUDE_RDV_OBJECT_DETECTOR_H

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>   

#include <darknet.h>

typedef struct _Object2D
{
	int class_num = -1 ;
	
	int x = 0 ;
	int y = 0 ;
	int w = 0 ;
	int h = 0 ;
	double score = 0 ;

	double min_depth = 0 ;
	double camera_center_distance = 0 ;
	
	int min_depth_pos_x = 0 ;
	int min_depth_pos_y = 0 ;

    bool operator < (const _Object2D& obj) const
    {
        return (score < obj.score);
    }
}Object2D ;

typedef struct _SortDepthObject2D
{
    inline bool operator() (const Object2D& struct1, const Object2D& struct2)
    {
        return (struct1.min_depth > struct2.min_depth);
    }
}SortDepthObject2D;

typedef struct _SortCenterDistanceObject2D
{
    inline bool operator() (const Object2D& struct1, const Object2D& struct2)
    {
        return (struct1.camera_center_distance > struct2.camera_center_distance);
    }
}SortCenterDistanceObject2D;




class CRdvObjectDetector
{
public:
	CRdvObjectDetector(const bool b_use_gaussian_pdf, std::string str_config_path, std::string str_weight_path, std::string str_meta_path, const double threshold=0.5) ;
	~CRdvObjectDetector() ;

	std::vector<Object2D> Run(cv::Mat input_image, const int sorting) ;

	void SetConfig_UseGaussianPdf(const bool value) ;
	bool GetConfig_UseGaussianPdf(void) ;
	
private :
	network *m_pNetwork;
	metadata m_Data;

	float m_f_threshold ;

	std::string m_str_ConfigPath ;
    std::string m_str_WeightsPath ;
    std::string m_str_MetaDataPath ;

	bool m_b_use_gaussian_pdf ;
	
	//darknet
	void MakeImage(cv::Mat m) ;
	image m_yolo_image ;

	cv::Mat m_gaussian_pdf ;
	cv::Mat MakeGaussianPdfImage(const int width, const int height, const double sigma, const bool b_debug=false) ;
};

#endif

