/*!
 * \author Tianyi Zhang
 * \version 0.1
 * \date 2019-01-30
 * \bug orb not working
 * \copyright MIT license
 * \mainpage Visual Odometry 
 * \section intro_sec Introduction
 * This is a feature-based Visual-odometry on DROPLab's SPHERE AUV stereo datasets
 * \section compile_sec Compilation
 * \subsection see readme
 * see readme
 */

#include <iostream>
#include <string>
#include <dirent.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "../include/extractor.hpp"
#include "../include/tracker.hpp"
#include "../include/stereoFrame.hpp"
#include "../include/loggor.hpp"
#include "../include/matcher.hpp"
#include "../include/displayer.hpp"

using namespace std;
using namespace cv;

Point2i maskLeftTop(168, 125); //!< Region of interest for features: top-left corner position
Size maskSize(1200, 930);      //!< Region of interest for features: width and height

const string FEATURETYPE = "akaze"; // "kaze" or "orb" or "akaze"
const string MATCHMETHOD = "BruteForce";
const string path = "../../MHLdata/2018_04_17_224525/selected/";
const char *leftPath = "../../MHLdata/2018_04_17_224525/selected/lg/";
const char *rightPath = "../../MHLdata/2018_04_17_224525/selected/rg/";
const char *CALIBFILEPATH = "../MHL_17042018.calib";

// const string path = "../../data/";
// const char *leftPath = "../../data/lg/";
// const char *rightPath = "../../data/rg/";
// const char *CALIBFILEPATH = "../6_FEB_TURTLEIMAGES.calib";

int main()
{
    DIR *dirpl, *dirpr;
    struct dirent *directoryl, *directoryr;
    vector<string> leftFileName, rightFileName;
    dirpl = opendir(leftPath);
    dirpr = opendir(rightPath);
    if (dirpl && dirpr)
    {
        while ((directoryl = readdir(dirpl)) != NULL && (directoryr = readdir(dirpr)) != NULL)
        {
            leftFileName.push_back(directoryl->d_name);
            rightFileName.push_back(directoryr->d_name);
        }
        closedir(dirpl);
        closedir(dirpr);
    }
    sort(leftFileName.begin(), leftFileName.end());
    sort(rightFileName.begin(), rightFileName.end());

    //initialize
    extractor extractor_(FEATURETYPE);
    matcher matcher_(MATCHMETHOD);

    //get img size
    string imgLeftFilename = path + "lg/" + leftFileName[2];
    Mat imgTest = cv::imread(imgLeftFilename, 0);

    // cout << imgTest.size() << endl;
    stereoFrame currentFrame_(extractor_, matcher_, CALIBFILEPATH, imgTest.size());
    stereoFrame lastFrame_(extractor_, matcher_, CALIBFILEPATH, imgTest.size());

    loggor log_;
    displayer displayer_;

    extractor_.setMask(imgTest.size(), maskLeftTop, maskSize);
    tracker tracker_(matcher_);
    int i = 1;
    while (i < 556)
    {
        string imgLeftFilename = path + "lg/" + leftFileName[i + 1];
        string imgRightFilename = path + "rg/" + rightFileName[i + 1];
        // cout << imgLeftFilename << " | " << imgRightFilename << endl;

        //readin img
        currentFrame_.camLeft.img = cv::imread(imgLeftFilename, 0);
        currentFrame_.camRight.img = cv::imread(imgRightFilename, 0);

        currentFrame_.processFrame();

        tracker_.runTracking(currentFrame_, lastFrame_);

        log_.logNow(tracker_.R, tracker_.T, currentFrame_.camRight.kp.size(), currentFrame_.camLeft.kp.size(), currentFrame_.camLeft.kpSelected.size(), currentFrame_.matches.size(), tracker_.kpTranslation.size());
        if (i != 1)
        {
            // displayer_.display4p(lastFrame_.camLeft, lastFrame_.camRight, currentFrame_.camLeft, currentFrame_.camRight);
            // displayer_.display(lastFrame_.camRight, currentFrame_.camRight, tracker_.goodMatchesLastCurrent);
        };

        lastFrame_ = currentFrame_;

        cout << "FRAME " << i << " PROCESSED" << endl;
        i++;
    }
}