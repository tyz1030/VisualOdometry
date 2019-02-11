#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/core.hpp>
#include "extractor.hpp"
#include "stereoFrame.hpp"
#include "matcher.hpp"

class tracker
{

public:
  tracker(matcher &matcher_);

  void depthEstimation(const std::vector<cv::KeyPoint> &kpLeft, const std::vector<cv::KeyPoint> &kpRight, std::vector<double> &depth);

  void gen3DPoint(const stereoFrame &frame, std::vector<cv::Point3d> &kp3d);

  void findRotation(const std::vector<cv::KeyPoint> &kpLast, const std::vector<cv::KeyPoint> &kpCurrent, const cv::Mat &P);

  void findTranslation();

  void findRTQtn();

  void findRTSVD();

  void runTracking(stereoFrame &currentFrame_, stereoFrame &lastFrame_);

  cv::Point3d MatPtMult(const cv::Mat M, const cv::Point3d &p);

  matcher &matcher_;

  cv::Mat translationDirection; //!< unit vector last2current translation

  cv::Mat R;     //!< last2current rotation matrix
  cv::Point3d T; //!< fitted last2current translation

  std::vector<cv::Point3d> kp3dLast;      //!< 3d point in last frame
  std::vector<cv::Point3d> kp3dCurrent;   //!< 3d point in current frame
  std::vector<cv::Point3d> kpTranslation; //!< translation of inliers

  std::vector<cv::DMatch> matchesLastCurrent; //!< last2current matches

private:
  cv::Mat mask_; //!< inlier mask
};

#endif