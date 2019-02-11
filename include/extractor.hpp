#ifndef EXTRACTOR_HPP
#define EXTRACTOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "camera.hpp"
// #include <opencv2/core/cvstd_wrapper.hpp>

class extractor
{

public:
  extractor(const std::string ft);
 
  void setMask(const cv::Size imgSize, const cv::Point2i maskLeftTop, const cv::Size maskSize);
 
  void extractFeatures(camera &camera_);

  std::string featureType; //!< feature type, "kaze" or "orb" or "akaze"

private:
  cv::Mat mask; //!< region of interest for feature detection
  cv::Ptr<cv::Feature2D> detector; //!< feature detector 
};

#endif