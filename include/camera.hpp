#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/core.hpp>

class camera
{

public:
  camera();

  void setParameters(const cv::Mat intri, const cv::Mat dist);

  void setFromPath(const char *calibFilePath, const std::string whichCam);

  cv::Mat img; //!< rectified image

  std::vector<cv::KeyPoint> kp;         //!< raw keypoints
  std::vector<cv::KeyPoint> kpSelected; //!< last2current matched keypoints

  cv::Mat descriptors; //!< raw descriptors
  cv::Mat desSelected; //!< left2right matched descriptors

  cv::Mat recR; //!< rectification transform
  cv::Mat recP; //!< projection matrix in rectified coordinate

  cv::Mat intriMat; //!< camera matrix
  cv::Mat distMat; //!< distortion matrix

private:
};

#endif