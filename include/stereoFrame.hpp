#ifndef STEREOFRAME_HPP
#define STEREOFRAME_HPP

#include <functional>

#include <opencv2/core.hpp>
#include "camera.hpp"
#include "extractor.hpp"
#include "matcher.hpp"

class stereoFrame
{

public:
  stereoFrame(const std::reference_wrapper<extractor> extractor_, const std::reference_wrapper<matcher> matcher_, const char *calibFilePath, const cv::Size size);

  void loadCalibFile(const char *calibFilePath, camera &camLeft, camera &camRight);

  void processFrame();

  void rectifyFrame();

  void extractFeatures();

  void matchFeatures();

  std::reference_wrapper<extractor> extractor_; //!< reference wrapper of extractor

  std::reference_wrapper<matcher> matcher_; //!< reference wrapper of matcher

  camera camLeft; //!< instance of left camera

  camera camRight; //!< instance of right camera

  std::vector<cv::DMatch> matches; //!< filtered left2right matches

  cv::Mat R; //!< rotation from 1st camerea to 2nd

  cv::Mat T; //!< translation from 1st camerea to 2nd

private:
};

#endif