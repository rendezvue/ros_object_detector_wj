#include "RdvObjectDetector.h"

CRdvObjectDetector::CRdvObjectDetector(const bool b_use_gaussian_pdf, std::string str_config_path, std::string str_weight_path, std::string str_meta_path, const double threshold) :
	m_pNetwork(NULL)
	,m_f_threshold(threshold)
	,m_str_ConfigPath(str_config_path)
    ,m_str_WeightsPath(str_weight_path)
    ,m_str_MetaDataPath(str_meta_path)
    ,m_b_use_gaussian_pdf(b_use_gaussian_pdf)
{
	printf("Init CRdvObjectDetector -- \n") ;

	m_yolo_image.w = 0 ;
	m_yolo_image.h = 0 ;
	m_yolo_image.c = 0 ;
	m_yolo_image.data = NULL ;

	//std::string ConfigPath = "../train/iforange.cfg";
    //std::string WeightsPath = "../train/iforange_last.weights";
    //std::string MetaDataPath = "../train/iforange.data";

	printf(" - load_network 1\n") ;
    //m_pNetwork = load_network((char *) m_str_ConfigPath.data(), (char *) m_str_WeightsPath.data(), 0);
    m_pNetwork = load_network_custom((char *) m_str_ConfigPath.data(), (char *) m_str_WeightsPath.data(), 0, 1) ;
	
	printf(" - load_network 2\n") ;
	
	printf(" - get_metadata 1\n") ;
	m_Data = get_metadata((char *) m_str_MetaDataPath.data());
	printf(" - get_metadata 2\n") ;
}

CRdvObjectDetector::~CRdvObjectDetector() 
{
	if( m_pNetwork )
	{
		free_network_ptr(m_pNetwork) ;
	}

	if( m_yolo_image.data )
	{
		free(m_yolo_image.data) ;
		m_yolo_image.data = NULL ;
	}
}

// Function to create Gaussian filter 
cv::Mat CRdvObjectDetector::MakeGaussianPdfImage(const int width, const int height, const double sigma, const bool b_debug)
{ 
	cv::Mat gaussian_pdf = cv::Mat::zeros(cv::Size(width, height), CV_64FC1) ;
	double* my_ptr = (double*)gaussian_pdf.data; 
	
    // intialising standard deviation to 1.0 
    double r, s = 2.0 * sigma * sigma; 
  
    // sum is for normalization 
    double sum = 0.0; 

	const double center_x = (double)width/2.0 ;
	const double center_y = (double)height/2.0 ;
	
	for( int y=0 ; y<height ; y++ )
	{
		int y_index = y*width ;
		
		for( int x=0 ; x<width ; x++ )
		{
			double _x = (double)x-center_x ;
			double _y = (double)y-center_y ;

			r = sqrt(_x * _x + _y * _y); 
			
			double data = (exp(-(r * r) / s)) / (M_PI * s); 

			my_ptr[y_index + x] = data; 

			sum += data ;
		}
	}

	for( int y=0 ; y<height ; y++ )
	{
		int y_index = y*width ;
		
		for( int x=0 ; x<width ; x++ )
		{
			my_ptr[y_index + x] /= sum ; 
		}
	}

	cv::normalize(gaussian_pdf, gaussian_pdf, 0, 1.0, cv::NORM_MINMAX);

	if( b_debug )
	{
		//debug	
		double minVal; 
		double maxVal; 
		cv::Point minLoc; 
		cv::Point maxLoc;	
		cv::minMaxLoc( gaussian_pdf, &minVal, &maxVal, &minLoc, &maxLoc );
		
		printf("max value = %f\n", maxVal) ;
		
		cv::Mat mat_debug ;
		cv::normalize(gaussian_pdf, mat_debug, 0, 255.0, cv::NORM_MINMAX);
		cv::imwrite("test_gaussian.png", mat_debug) ;
	}
	
	return gaussian_pdf ;
}

std::vector<Object2D> CRdvObjectDetector::Run(cv::Mat input_image, const int sorting)
{
	std::vector<Object2D> vec_ret_objects ;
		
	printf("Object Detector Start: Image size : %d x %d, threshold = %f\n", input_image.cols, input_image.rows, m_f_threshold) ;
	printf("Object Detector Start: Network Image size : %d x %d\n", m_pNetwork->w, m_pNetwork->h) ;

	cv::Mat resize_image ;
#if 1
	float resize_scale = 1.0 ;
	cv::Rect rect_copy_from, rect_copy_to ;
	if( input_image.cols != m_pNetwork->w || input_image.rows != m_pNetwork->h )
	{
		float resize_scale_w = (float)m_pNetwork->w / (float)input_image.cols  ;
		float resize_scale_h = (float)m_pNetwork->h / (float)input_image.rows ;

		resize_scale = cv::min(resize_scale_w, resize_scale_h) ;

		cv::Mat input_resize_image ;
		cv::resize(input_image, input_resize_image, cv::Size(), resize_scale, resize_scale) ; 
		
		resize_image = cv::Mat::zeros(cv::Size(m_pNetwork->w, m_pNetwork->h), CV_8UC3) ;

		rect_copy_from = cv::Rect(0,0,input_resize_image.cols, input_resize_image.rows) ;
		rect_copy_to = cv::Rect(0,0,resize_image.cols, resize_image.rows) ;

		rect_copy_to.x = (resize_image.cols - input_resize_image.cols)/2.0 ;
		if( rect_copy_to.x < 0 )	rect_copy_to.x = 0 ;
		rect_copy_to.y = (resize_image.rows - input_resize_image.rows)/2.0 ;
		if( rect_copy_to.y < 0 )	rect_copy_to.y = 0 ;
		rect_copy_to.width = rect_copy_from.width ;
		if( rect_copy_to.x + rect_copy_to.width > resize_image.cols )
		{
			rect_copy_to.width = rect_copy_to.width - rect_copy_to.x ;
			rect_copy_from.width = rect_copy_to.width ;
		}
		rect_copy_to.height = rect_copy_from.height ;
		if( rect_copy_to.y + rect_copy_to.height > resize_image.rows )
		{
			rect_copy_to.height = rect_copy_to.height - rect_copy_to.y ;
			rect_copy_from.height = rect_copy_to.height ;
		}

		input_resize_image(rect_copy_from).copyTo(resize_image(rect_copy_to)) ;
		
	}
	else
	{
		resize_image = input_image ;
	}
#else
	resize_image = input_image ;
#endif

	//Gaussian PDF
	if( m_gaussian_pdf.empty() || m_gaussian_pdf.cols != resize_image.cols || m_gaussian_pdf.rows != resize_image.rows )
	{
		m_gaussian_pdf = MakeGaussianPdfImage(resize_image.cols, resize_image.rows, (double)std::min(resize_image.cols, resize_image.rows), true) ;
	}
	double* ptr_gaussian_pdf = (double*)m_gaussian_pdf.data; 
	
	//image Image = mat_to_image(input_image) ;
	MakeImage(resize_image) ;
	
	network_predict_image(m_pNetwork, m_yolo_image);

	int nCount = 0 ;
	detection *pDetection = get_network_boxes(m_pNetwork, m_yolo_image.w, m_yolo_image.h, m_f_threshold, m_f_threshold, nullptr, 0, &nCount, 0);

	printf("Object Detector 1: nCount = %d\n", nCount) ;

	do_nms_obj(pDetection, nCount, m_Data.classes, m_f_threshold);//第二步：do_nms_obj

	for (size_t j = 0; j < nCount; j++)
    {
        for (size_t i = 0; i < m_Data.classes; i++)
        {
            if (pDetection[j].prob[i] > 0)
            {
            	box b = pDetection[j].bbox;
				//printf("Detect [%d] score(%f), box(%f, %f, %f, %f)\n", (int)j, pDetection[j].prob[i], b.x, b.y, b.w, b.h) ;
				
				int top = (b.y - b.h / 2);
                int bot = (b.y + b.h / 2);
                int left  = (b.x - b.w / 2);
                int right  = (b.x + b.w / 2);

				cv::Point pt_center = cv::Point(left+(right-left)/2,top+(bot-top)/2)  ;

				double score = pDetection[j].prob[i] ;
				double mask_value = 1.0 ;
				if( !m_gaussian_pdf.empty() && m_b_use_gaussian_pdf )
				{
					int _x = pt_center.x ;
					int _y = pt_center.y ;

					if( _x < 0 ) _x = 0 ;
					if( _x >= m_gaussian_pdf.cols ) _x = m_gaussian_pdf.cols ;
					if( _y < 0 ) _y = 0 ;
					if( _y >= m_gaussian_pdf.rows ) _y = m_gaussian_pdf.rows ;
					
					mask_value = ptr_gaussian_pdf[_y * m_gaussian_pdf.cols + _x] ;
				}
				
				Object2D object ;
				object.class_num = i ;
				object.x = left ;
				object.y = top ;
				object.w = right - left ;
				object.h = bot - top ;
				object.score = score * mask_value ;
				float dx = (float)pt_center.x - (float)input_image.cols/2.0 ;
				float dy = (float)pt_center.y - (float)input_image.rows/2.0 ;
				object.camera_center_distance = sqrt(dx*dx + dy*dy) ; 

#if 1
				if( resize_scale != 1.0 )
				{
					object.x -= rect_copy_to.x ;
					object.y -= rect_copy_to.y ;
					
					object.x = (float)object.x / resize_scale;
					object.y = (float)object.y / resize_scale;
					object.w = (float)object.w / resize_scale;
					object.h = (float)object.h / resize_scale;

					pt_center.x -= rect_copy_to.x ;
					pt_center.y -= rect_copy_to.y ;
					pt_center.x = (float)pt_center.x / resize_scale ;
					pt_center.y = (float)pt_center.y / resize_scale ;
					
					float dx = (float)pt_center.x - (float)input_image.cols/2.0 ;
					float dy = (float)pt_center.y - (float)input_image.rows/2.0 ;
					object.camera_center_distance = sqrt(dx*dx + dy*dy) ; 
				}
#endif
				vec_ret_objects.push_back(object) ;
            }
        }
    }

	free_detections(pDetection, nCount);
	//free_image(Image);

	//sorting
	if( sorting )
	{
		std::sort(vec_ret_objects.begin(), vec_ret_objects.end());
	}
			
	return vec_ret_objects ;
}


void CRdvObjectDetector::MakeImage(cv::Mat mat)
{
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();

	if( m_yolo_image.data )
	{
		if( m_yolo_image.w != w || m_yolo_image.h != h || m_yolo_image.c != c )
		{
			free(m_yolo_image.data) ;
			m_yolo_image.data = NULL ;
		}
	}

	if( m_yolo_image.data == NULL )
	{
		m_yolo_image.w = w ;
		m_yolo_image.h = h ;
		m_yolo_image.c = c ;
		
		const int nmemb = m_yolo_image.h * m_yolo_image.w * m_yolo_image.c;
		m_yolo_image.data = (float *)calloc(nmemb, sizeof(float));
	    if(!m_yolo_image.data) 
		{
	        fprintf(stderr, "Calloc error - possibly out of CPU RAM \n");
		    exit(EXIT_FAILURE);
	    }
	    memset(m_yolo_image.data, 0, nmemb * sizeof(float));
	}
	
    unsigned char *data = (unsigned char *)mat.data;
    int step = mat.step;

	int y_step = 0 ;
	int yxwidth = 0 ;
	int wxh = w*h ;
	int kxwxh = 0 ;
	int y_step_p_k = 0 ;
	int kxwxh_p_yxwidth= 0 ;

	int c_minus_1 = (c-1) ;
	printf("MakeImage : source channel inverse\n") ;
	
    for (int y = 0; y < h; ++y) 
	{
		y_step = y*step ;
		yxwidth = y*w ;
		
        for (int k = 0; k < c; ++k)
		{	
			//target
			kxwxh = k*wxh ;
			kxwxh_p_yxwidth = kxwxh + yxwidth ;

			//source
			y_step_p_k = y_step+(c_minus_1-k);	//(c_minus_1-k) : channel inverse
			
            for (int x = 0; x < w; ++x) 
			{
                m_yolo_image.data[kxwxh_p_yxwidth + x] = (float)(data[y_step_p_k + x*c]) / 255.0f;
            }
        }
    }
    //return im;
}

void CRdvObjectDetector::SetConfig_UseGaussianPdf(const bool value)
{
	m_b_use_gaussian_pdf = value ;
}

bool CRdvObjectDetector::GetConfig_UseGaussianPdf(void)
{
	return m_b_use_gaussian_pdf ;
}

