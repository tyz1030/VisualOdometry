#include "../include/displayer.hpp"

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include "../include/camera.hpp"

using namespace std;
using namespace cv;

/*!
 * \brief displayer constructor
 */
displayer::displayer(){};

/*!
 * \brief This function displays the left2right matching with Opencv drawMatches function.
 * 
 * \param[in] cam1 an instance of camera the image of which will be displayed in the left
 * \param[in] cam2 an instance of camera the image of which will be displayed in the right
 * \param[in] thismatch std::vector<cv::DMatch> type matches of keypoints in cam1 and cam2 
 */
void displayer::display(const camera &cam1, const camera &cam2, const vector<DMatch> &thismatch)
{
    counter = 0;
    Mat img_match;
    // cout << cam1.img.size() << cam1.kpSelected.size() << thismatch.size() << endl;
    drawMatches(cam1.img, cam1.kp, cam2.img, cam2.kp, thismatch, img_match);

    namedWindow("all matchings", WINDOW_NORMAL);
    imshow("all matchings", img_match);
    resizeWindow("all matchings", 1920, 800);
    waitKey(200);
};

/*!
 * \brief This function displays the left2right matching of last frame, left2right matching of current frame
 * and last right to current right matching all together in a 4grid image
 * 
 * \param[in] camlt an instance of camera the image of which will be displayed in the left top
 * \param[in] camrt an instance of camera the image of which will be displayed in the right top
 * \param[in] camlb an instance of camera the image of which will be displayed in the left left bottom
 * \param[in] camrb an instance of camera the image of which will be displayed in the right right bottom
 */
void displayer::display4p(const camera &camlt, const camera &camrt, const camera &camlb, const camera &camrb)
{
    counter++;
    Mat left2, right2, all4;
    vconcat(camlt.img, camlb.img, left2);
    vconcat(camrt.img, camrb.img, right2);
    hconcat(left2, right2, all4);
    cvtColor(all4, all4c, COLOR_GRAY2RGB);
    for (int i = 1; i < camlb.kpSelected.size(); i++)
    {
        line(all4c, camlt.kpSelected[i].pt, Point2d(camrt.kpSelected[i].pt.x + 1536, camrt.kpSelected[i].pt.y), Scalar(222, 22, 22), 2, 8, 0);
        line(all4c, Point2d(camrt.kpSelected[i].pt.x + 1536, camrt.kpSelected[i].pt.y), Point2d(camrb.kpSelected[i].pt.x + 1536, camrb.kpSelected[i].pt.y + 1180), Scalar(22, 222, 22), 2, 8, 0);
        line(all4c, Point2d(camlb.kpSelected[i].pt.x, camlb.kpSelected[i].pt.y + 1180), Point2d(camrb.kpSelected[i].pt.x + 1536, camrb.kpSelected[i].pt.y + 1180), Scalar(0, 22, 222), 2, 8, 0);
    }
    namedWindow("all matchings", WINDOW_NORMAL);
    imshow("all matchings", all4c);
    resizeWindow("all matchings", 1536, 1180);
    // cout << camlt.kpSelected2[0].pt << camrt.kpSelected2[0].pt << endl;
    // imwrite("../data" + to_string(counter) + ".png", all4c);
    waitKey(0);
};