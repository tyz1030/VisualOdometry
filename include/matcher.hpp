#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "camera.hpp"

class matcher
{
public:
  matcher(const std::string matchMethod);

  void matchDescriptors(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &thismatch);

  void matchFilter(const std::vector<cv::DMatch> &rawMatches, const double threshold, std::vector<cv::DMatch> &goodMatches);

  void epipolarRejector(const std::vector<cv::DMatch> &goodMatches, camera &camLeft, camera &camRight, std::vector<cv::DMatch> &horizontalMatches);

  void biDirecRejector(const std::vector<cv::DMatch> &rawMatches, const std::vector<cv::DMatch> &rawMatches2, std::vector<cv::DMatch> &filteredMatches);

  void updateMatchPoints(camera &camLeft, camera &camRight, const std::vector<cv::DMatch> &matches);

  void updateMatchPoints2(camera &camLeftLast, camera &camRightLast, camera &camLeftCurrent, camera &camRightCurrent, const std::vector<cv::DMatch> &matches, const std::vector<cv::DMatch> &matches2, const std::vector<cv::DMatch> &matches3);

  cv::Ptr<cv::DescriptorMatcher> thismatcher; //!< descriptor matcher

private:
  std::string matchMethod; //!< matching method :`BruteForce` (it uses L2 ), `BruteForce-L1`, `BruteForce-Hamming`, `BruteForce-Hamming(2)`, `FlannBased`
};

#endif