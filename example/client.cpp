#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "RdvObjectDetector.h"
#include "ros/ros.h"
#include "ros_object_detector/SrvEnsemble.h"
#include "ros_object_detector/yolo.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sstream>
#include <message_filters/sync_policies/exact_time.h>
#include <iostream>
#include "rb_test_ui/robot_srv.h"

int flag;
double xMin, yMin, xMax, yMax;
double xMin1, yMin1, xMax1, yMax1;
double xMin2, yMin2, xMax2, yMax2;

double final_X, final_Y;
bool run_service(rb_test_ui::robot_srv::Request &req, rb_test_ui::robot_srv::Response &res)
{

   flag = req.flag;

   res.xMax = xMax;
   res.xMin = xMin;
   res.yMax = yMax;
   res.yMin = yMin;
   res.final_X = final_X;
   res.final_Y = final_Y;

   res.xMax1 = xMax1;
   res.xMin1 = xMin1;
   res.yMax1 = yMax1;
   res.yMin1 = yMin1;

   res.xMax2 = xMax2;
   res.xMin2 = xMin2;
   res.yMax2 = yMax2;
   res.yMin2 = yMin2;

   //res.xMax = 999;
   //res.xMin = 999;
   //res.yMax = 333;
   //res.yMin = 333;

   printf("=======transfer ROI data TO robot mainwindow===========");

   return true;

}

class Node
{
public:
   Node()
   {

      image.subscribe(nh_, "/camera/color/image_raw", 10);
      //image_depth.subscribe(nh_, "/camera/depth/image_rect_raw", 1);
      image_depth.subscribe(nh_, "/camera/depth/image_rect_raw", 10);
      camera_INFO.subscribe(nh_, "/camera/color/camera_info", 10);
      sync_.reset(new Sync(MySyncPolicy(10), image, image_depth, camera_INFO));
      sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2, _3));
      cnt = 0;
      //WSADATA wsaData;
      //ros::ServiceServer service_server = nh_.advertiseService("ros_srv", run_service) ;

   }






   bool service_call(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &image_depth, const sensor_msgs::CameraInfoConstPtr &camera_INFO)
   {
      ros::NodeHandle n;

      //cv::Mat input_image = cv::imread(image_path) ;
      cv::Mat input_image ;
      cv::Mat input_image_depth;
      cv::Mat im2;
      cv_bridge::CvImagePtr cv_ptr;
      cv_bridge::CvImagePtr cv_ptr_d;

      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv_ptr_d = cv_bridge::toCvCopy(image_depth, sensor_msgs::image_encodings::TYPE_32FC1);
      input_image = cv_ptr->image;
      input_image_depth = cv_ptr_d->image;

      //float temp = input
      cv::imwrite("result4.png", input_image_depth) ;


      int w2 = input_image_depth.cols;
      int h2 = input_image_depth.rows;

      int width = input_image.cols;
      int height = input_image.rows;
      int bpp = input_image.channels();
      fprintf(stderr, "w2:%d, h2:%d, w1:%d, h1:%d,\n", w2, h2, width, height );
      int imagesize = width * height * bpp;
      float xx = 0;
      float yy = 0;
      float zz = 0;

      //instric parameter
      float fx = 608.3635;
      float fy = 608.3475;
      float cx = 325.998;
      float cy = 232.4725;
      //float
      double BB1_x = 0;
      double BB1_y = 0;
      double BB2_x = 0;
      double BB2_y = 0;
      double BB3_x = 0;
      double BB3_y = 0;
      double BB4_x = 0;
      double BB4_y = 0;
      double BB11_x = 0;
      double BB11_y = 0;
      double BB22_x = 0;
      double BB22_y = 0;
      double BB33_x = 0;
      double BB33_y = 0;
      double BB44_x = 0;
      double BB44_y = 0;
      double cen_x = 0;
      double cen_y = 0;
      double cen_z = 0;

      fprintf(stderr, "ros Object Detector client\n") ;
      ros::ServiceClient client = n.serviceClient<ros_object_detector::SrvEnsemble>("ros_object_detector_service");
      ros_object_detector::SrvEnsemble srv;
      srv.request.str_cmd = "run";
      srv.request.width = width;
      srv.request.height = height;
      srv.request.bpp = bpp;

      //memcpy(&srv.request.data[0], input_image.data, imagesize);
      //std::copy(input_image.data, imagesize, &srv.request.data[0]);

      for ( int i = 0 ;i < imagesize ; i++ ) // copy "image buffer" -> "ros msg vector"
      {
         srv.request.data.push_back(input_image.data[i]);
      }

      if ( client.call(srv) )
      {
         fprintf(stderr, "Service call success\n");

         //cen_z,xmin,ymin,xmax,ymax,score
         double arr[srv.response.result.size()][6];

         double min_z = 0;
         for ( int i = 0 ;i < srv.response.result.size(); i++)
         {
            ros_object_detector::yolo ret_data = srv.response.result[i];

            //fprintf(stderr,"i-th: %d x:%d, y:%d, w:%d, h:%d, score:%f\n", i,ret_data.x, ret_data.y, ret_data.width, ret_data.height, ret_data.score );


            BB1_x = (double)ret_data.x;
            BB1_y = (double)ret_data.y;
            BB2_x = (double)ret_data.x + ret_data.width;
            BB2_y = (double)ret_data.y;
            BB3_x = (double)ret_data.x + ret_data.width;
            BB3_y = (double)ret_data.y + ret_data.height;
            BB4_x = (double)ret_data.x;
            BB4_y = (double)ret_data.y + ret_data.height;

            float depth_data1 = input_image_depth.at<float>(BB1_y, BB1_x) ;
            float depth_data2 = input_image_depth.at<float>(BB2_y, BB2_x) ;
            float depth_data3 = input_image_depth.at<float>(BB3_y, BB3_x) ;
            float depth_data4 = input_image_depth.at<float>(BB4_y, BB4_x) ;
            //	fprintf(stderr,"1)xx:%f \n yy:%f \n",BB1_x,BB1_y);
            //	fprintf(stderr,"1)xx:%f \n yy:%f \n",BB2_x,BB2_y);
            //	fprintf(stderr,"1)xx:%f \n yy:%f \n",BB3_x,BB3_y);
            //	fprintf(stderr,"1)xx:%f \n yy:%f \n\n\n\n",BB4_x,BB4_y);

            BB11_x = (BB1_x - cx) / fx * (double)(depth_data1 * 0.001);
            BB11_y = (BB1_y - cy) / fy * (double)(depth_data1 * 0.001);
            BB22_x = (BB2_x - cx) / fx * (double)(depth_data2 * 0.001);
            BB22_y = (BB2_y - cy) / fy * (double)(depth_data2 * 0.001);
            BB33_x = (BB3_x - cx) / fx * (double)(depth_data3 * 0.001);
            BB33_y = (BB3_y - cy) / fy * (double)(depth_data3 * 0.001);
            BB44_x = (BB4_x - cx) / fx * (double)(depth_data4 * 0.001);
            BB44_y = (BB4_y - cy) / fy * (double)(depth_data4 * 0.001);

            /*BB11_x = (BB1_x-cx)/fx*0.6;
            BB11_y = (BB1_y-cy)/fy*0.6;
            BB22_x = (BB2_x-cx)/fx*0.6;
            BB22_y = (BB2_y-cy)/fy*0.6;
            BB33_x = (BB3_x-cx)/fx*0.6;
            BB33_y = (BB3_y-cy)/fy*0.6;
            BB44_x = (BB4_x-cx)/fx*0.6;
            BB44_y = (BB4_y-cy)/fy*0.6;*/

            cen_z = input_image_depth.at<float>((BB1_y + (BB3_y - BB1_y) / 2), (BB1_x + (BB2_x - BB1_x) / 2)) ;
            fprintf(stderr, "i-th: %d x:%d, y:%d, w:%d, h:%d, score:%f z:%f\n", i, ret_data.x, ret_data.y, ret_data.width, ret_data.height, ret_data.score, cen_z);
            xMin = BB11_x;
            yMin = BB11_y;
            xMax = BB22_x;
            yMax = BB33_y;
            arr[i][0] = cen_z;
            arr[i][1] = BB11_x;
            arr[i][2] = BB11_y;
            arr[i][3] = BB22_x;
            arr[i][4] = BB33_y;
            arr[i][5] = ret_data.score;

            fprintf(stderr, "z:%f\n, z:%f\n, z:%f\n, z:%f\n", depth_data1, depth_data2, depth_data3, depth_data4);


            final_X = ((BB1_x + (BB2_x - BB1_x) / 2) - cx) / fx * cen_z; //sawing cutter에서 사용 
            final_Y = ((BB1_y + (BB3_y - BB1_y) / 2) - cy) / fy * cen_z;
         }
         min_z = arr[0][0];


         for (int i = 0;i < srv.response.result.size();i++)
         {
            if (min_z >= arr[i][0] && arr[i][5] > 0.98)
            {
               min_z = arr[i][0];
               xMin = arr[i][1];
               yMin = arr[i][2];
               xMax = arr[i][3];
               yMax = arr[i][4]; //test
            }


         }
         printf("Min_z : %f", min_z);
         fprintf(stderr, "xmin:%f, ymin:%f, xmax:%f, ymax:%f\n", xMin, yMin, xMax, yMax);
         /* for(int i=0;i<50;i++)
          	{
         for(int j=0;j<50;j++)
         	{
         	fprintf(stderr,"z : %f\n ",input_image_depth.at<float>(200+i,300+j)	)	;  

         	}
          	}*/

      }
      else
      {
         fprintf(stderr, "Service Failed\n");
      }




   }




   //YOLO이미지를 받아오는 callback 함수
   void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &image_depth, const sensor_msgs::CameraInfoConstPtr &camera_INFO)
   {
      printf("yolo_CB");

      cv::Mat raw_image;
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      raw_image = cv_ptr->image;


      cv::imwrite("result2.png", raw_image) ;
      ros::NodeHandle nh_("~");
      ros::NodeHandle nh;
      std::string str_test_image_path_object;
      nh_.getParam("test_image_path_for_object", str_test_image_path_object);
      std::string str_image_path = str_test_image_path_object;




      printf("\nserver_call before\n");
      service_call(image, image_depth, camera_INFO);
      //	printf("server_call after\n");

      //ros::Duration(3).sleep();

      printf("ready srv server to robot");




   }


   ~Node()
   {}


private:
   ros::NodeHandle nh_;

   message_filters::Subscriber<sensor_msgs::Image> image;
   message_filters::Subscriber<sensor_msgs::Image> image_depth;
   message_filters::Subscriber<sensor_msgs::CameraInfo> camera_INFO;
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
   typedef message_filters::Synchronizer<MySyncPolicy> Sync;
   boost::shared_ptr<Sync> sync_;

public:
   int cnt;


};




int main(int argc, char **argv)
{
   ros::init(argc, argv, "ros_object_detector_client") ;

   Node synchronizer;
   ros::NodeHandle nh;
   ros::ServiceServer service_server = nh.advertiseService("ros_srv", run_service) ;

   ros::spin();
   return EXIT_SUCCESS;
}


