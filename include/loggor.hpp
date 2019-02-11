#ifndef LOGGOR_HPP
#define LOGGOR_HPP

#include <fstream>

#include <opencv2/core.hpp>

class loggor
{
  public:
    loggor();
    
    void logNow(cv::Mat Rotation, cv::Point3d Translation, int a, int b, int c, int d, int e);

    std::ofstream outfile;

  private:
    bool FIRSTFRAME;
};
#endif