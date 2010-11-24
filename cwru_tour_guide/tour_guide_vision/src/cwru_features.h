
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
    cv::Point3d calcRay(double yaw, double pitch, double roll); //calculates the unit vector from camera to robot in map frame
    
    
    cv::Point3d calcPosition(double r, const cv::Mat &camera_frame_offset); //calculates the unit vector from camera to robot in map frame
    
    cv::Point3d lastCalcedRay;
    cv::Point3d lastCalcedPosition;
    
    void print();
    
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
    cv::Point3d calc_least_squares_position(double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset);
    
    void add(const RawFeature &add_me);
    void merge(const FeatureManager & merger);
    int numFeatures();
    void print();
    
    std::vector<RawFeature> raw_features;
  private:
};
#endif
