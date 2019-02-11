#include "../include/camera.hpp"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;
/*!
 * \brief camera constructor
 */
camera::camera(){};

/*!
 * \brief This function sets camera intrinics and distortion parameters.
 * 
 * \param[in] intri cv::Mat type intrinsic matrix
 * \param[in] dist cv::Mat type distortion matrix
 */
void camera::setParameters(const Mat intri, const Mat dist)
{
    intriMat = intri;
    distMat = dist;
};

/*!
 * \brief This function sets camera intrinics and distortion parameters.
 * 
 * \param[in] calibFilePath char type pointer to calibration file path
 * \param[in] whichCam "left" or "right" indicating which calibration data to load 
 */
void camera::setFromPath(const char *calibFilePath, const string whichCam)
{
    vector<double> val;
    ifstream camFile;
    camFile.open(calibFilePath);
    string line, elem;
    while (getline(camFile, line))
    {
        stringstream ss(line);
        while (getline(ss, elem, ' '))
        {
            val.push_back(stod(elem));
            // cout << elem << endl;
        }
    };

    int i;
    
    if (whichCam == "left")
    {
        i = 0;
    }
    else if (whichCam == "right")
    {
        i = 28;
    }
    else
    {
        cout << "whichCam not found" << endl;
    };

    i += 3;
    Mat intriLeft = (Mat_<double>(3, 3) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4], val[i + 5], val[i + 6], val[i + 7], val[i + 8]);
    i += 9;
    Mat distLeft = (Mat_<double>(5, 1) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4]);
};