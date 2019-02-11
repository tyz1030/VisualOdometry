#include "../include/stereoFrame.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <functional>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "../include/camera.hpp"
#include "../include/extractor.hpp"
#include "../include/matcher.hpp"

using namespace std;
using namespace cv;

/*!
 * \brief stereoFrame constructor
 * 
 * \param[in] extractor_ reference wrapper of instance extractor
 * \param[in] matcher_ reference wrapper of instance matcher
 * \param[in] calibFilePath char type pointer to calibration file path
 * \param[in] size cv::Size type image size
 * \details This fun initialize the stereoFrame. This constructor initialize the member camLeft,  
 * camRight, extractor_ and matcher_. This constructor load stereo parameters and parameters for each camera.  
 */
stereoFrame::stereoFrame(const reference_wrapper<extractor> extractor_, const reference_wrapper<matcher> matcher_, const char *calibFilePath, const Size size)
    : camLeft(), camRight(), extractor_(extractor_), matcher_(matcher_)
{
    Mat Q;
    this->loadCalibFile(calibFilePath, camLeft, camRight);
    stereoRectify(camLeft.intriMat, camLeft.distMat, camRight.intriMat, camRight.distMat, size, R, T, camLeft.recR, camRight.recR, camLeft.recP, camRight.recP, Q);
};

/*!
 * \brief This function load parameter from files
 * 
 * \param[in] calibFilePath char type pointer to calibration file path
 * \param camLeft instance of left camera
 * \param camRight instance of right camera
 */
void stereoFrame::loadCalibFile(const char *calibFilePath, camera &camLeft, camera &camRight)
{
    std::vector<double> val;
    ifstream camFile;
    camFile.open(calibFilePath);
    string line, elem;
    while (getline(camFile, line))
    {
        stringstream ss(line);
        while (getline(ss, elem, ' '))
        {
            val.push_back(stod(elem));
        }
    };

    int i = 3;
    Mat intriLeft = (Mat_<double>(3, 3) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4], val[i + 5], val[i + 6], val[i + 7], val[i + 8]);
    i = 31;
    Mat intriRight = (Mat_<double>(3, 3) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4], val[i + 5], val[i + 6], val[i + 7], val[i + 8]);
    i = 12;
    Mat distLeft = (Mat_<double>(5, 1) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4]);
    i = 40;
    Mat distRight = (Mat_<double>(5, 1) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4]);
    i = 45;
    R = (Mat_<double>(3, 3) << val[i], val[i + 1], val[i + 2], val[i + 3], val[i + 4], val[i + 5], val[i + 6], val[i + 7], val[i + 8]);
    i = 54;
    T = (Mat_<double>(3, 1) << val[i], val[i + 1], val[i + 2]);
    // cout << intriLeft << intriRight << distLeft << distRight << R << T << endl;

    camRight.setParameters(intriRight, distRight);
    camLeft.setParameters(intriLeft, distLeft);
};

/*!
 * \brief This function rectifies stereo image
 * 
 * \details This function reprojects and undistorts left and right camera img
 */
void stereoFrame::rectifyFrame()
{
    Mat rmap[2][2];
    initUndistortRectifyMap(camLeft.intriMat, camLeft.distMat, camLeft.recR, camLeft.recP(Range(0,3), Range(0,3)), camLeft.img.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(camRight.intriMat, camRight.distMat, camRight.recR, camRight.recP(Range(0,3), Range(0,3)), camRight.img.size(), CV_16SC2, rmap[1][0], rmap[1][1]);
    remap(camLeft.img, camLeft.img, rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(camRight.img, camRight.img, rmap[1][0], rmap[1][1], INTER_LINEAR);

    // imshow("imgLeftRec", newleft);
};

/*!
 * \brief This function call extractor_ to extract features in left and right camera img
 */
void stereoFrame::extractFeatures()
{
    extractor_.get().extractFeatures(camLeft);
    extractor_.get().extractFeatures(camRight);
};

/*!
 * \brief This function call matcher_ to match features in left and right camera img
 */
void stereoFrame::matchFeatures()
{
    vector<DMatch> rawMatches, goodMatches;

    matcher_.get().matchDescriptors(camLeft.descriptors, camRight.descriptors, rawMatches);

    matcher_.get().matchFilter(rawMatches, 0.5, goodMatches);

    matcher_.get().epipolarRejector(goodMatches, camLeft, camRight, matches);

    matcher_.get().updateMatchPoints(camLeft, camRight, matches);
};

/*!
 * \brief This function run the rectfier, extractor and matcher.
 */
void stereoFrame::processFrame()
{
    this->rectifyFrame();

    this->extractFeatures();

    this->matchFeatures();
};