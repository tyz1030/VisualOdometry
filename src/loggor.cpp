#include "../include/loggor.hpp"

#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

loggor::loggor() : outfile("../logfile.csv") { FIRSTFRAME = true; };

void loggor::logNow(Mat Rotation, Point3d Translation, int a, int b, int c, int d, int e)
{
    if (FIRSTFRAME)
    {
        FIRSTFRAME = false;
    }
    else
    {
        outfile << Rotation.at<double>(0,0) << "," << Rotation.at<double>(0,1) << "," << Rotation.at<double>(0,2) << ",";
        outfile << Rotation.at<double>(1,0) << "," << Rotation.at<double>(1,1) << "," << Rotation.at<double>(1,2) << ",";
        outfile << Rotation.at<double>(2,0) << "," << Rotation.at<double>(2,1) << "," << Rotation.at<double>(2,2) << ",";
        outfile << Translation.x << "," << Translation.y << "," << Translation.z << ",";
        outfile << a << ","<< b << ","<< c << ","<< d << ","<< e << endl;
    };
};