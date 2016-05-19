#include <opencv_linemod_test/opencv_linemod_test.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/rgbd/rgbd.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_linemod_test");
  ros::NodeHandle nh;
  
  
  cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
   
  return 0;
}