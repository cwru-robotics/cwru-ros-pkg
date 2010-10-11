#ifndef CWRU_FEATURES_H
#define CWRU_FEATURES_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

class RawFeature{
  public:
    RawFeature();
    RawFeature(double x, double y, double theta, cv::Point2d pixel,cv::Point3d ray);
    ~RawFeature();
    cv::Point3d calcRay(const cv::Mat& camera2robot_axis,double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset); //calculates the unit vector from camera to robot in map frame
    
    cv::Point3d lastCalcedRay;
    
    double r_value;
    friend class FeatureManager;
    
  private:
    double x;
    double y;
    double theta;
    cv::Point2d pixel;
    cv::Point3d ray;
};

class FeatureManager{
  public:
    FeatureManager();
    ~FeatureManager();
    cv::Point3d currentMean;
    
    //assigns the current mean and returns the error of the least squares value
    double calc_least_squares_position(const cv::Mat& camera2robot_axis,double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset);
    
    void add(const RawFeature &add_me);
    void merge(const FeatureManager & merger);
    int numFeatures();
  private:
    std::vector<RawFeature> raw_features;
};
#endif
