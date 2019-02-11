#include "../include/extractor.hpp"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "../include/camera.hpp"

using namespace std;
using namespace cv;

/*!
 * \brief extractor constructor
 * 
 * \param[in] ft feature type ("kaze", "akaze", "orb")
 * \details This fun initialize the extractor of designated type
 */
extractor::extractor(const string ft)
{
    featureType = ft;
    if (featureType == "orb")
    {
        int numFeatures = 10000;
        float scaleFactor = 1.0f;
        int numLayers = 1;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        int patchSize = 31;
        int fastThreshold = 5;
        detector = ORB::create(numFeatures, scaleFactor, numLayers, edgeThreshold, firstLevel, WTA_K, ORB::FAST_SCORE, patchSize, fastThreshold);
    }
    else if (featureType == "kaze")
    {
        detector = KAZE::create(false, false, 0.0001f, 2, 3);
    }
    else if (featureType == "akaze")
    {
        detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00018f, 2, 3, KAZE::DIFF_PM_G2);
    };
};

/*!
 * \brief this function sets mask for extractor
 * 
 * \param[in] imgSize image size
 * \param[in] maskLeftTop mask left top point
 * \param[in] maskSize mask size
 */
void extractor::setMask(const Size imgSize, const Point2i maskLeftTop, const Size maskSize)
{
    // generate a mask for feature detection
    mask = Mat::zeros(imgSize.height, imgSize.width, CV_8U);
    mask(Rect(maskLeftTop.x, maskLeftTop.y, maskSize.width, maskSize.height)) = 1;
};

/*!
 * \brief this function detect keypoints and descriptors
 * 
 * \param[in] camera an instance of camera
 */
void extractor::extractFeatures(camera &camera)
{
    detector->detectAndCompute(camera.img, mask, camera.kp, camera.descriptors);
    // cout << "size of KP is " << camera.kp.size() << endl;
};