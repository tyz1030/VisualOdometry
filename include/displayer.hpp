#ifndef DISPLAYER_HPP
#define DISPLAYER_HPP

#include <opencv2/core.hpp>
#include "camera.hpp"

class displayer
{
public:
  displayer();

  void display(const camera &cam1, const camera &cam2, const std::vector<cv::DMatch> &thismatch);
  void display4p(const camera &camlt, const camera &camrt, const camera &camlb, const camera &camrb);

private:
  cv::Mat all4c; //!< concatencated output imag
  int counter; //!< counter for output file naming 
};

#endif