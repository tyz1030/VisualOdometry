#include "../include/tracker.hpp"

#include <iostream>
#include <numeric> // std::accumulate
#include <math.h>  /* acos */

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "../include/extractor.hpp"
#include "../include/stereoFrame.hpp"
#include "../include/matcher.hpp"

using namespace std;
using namespace cv;

/*!
 * \brief tracker constructor
 * 
 * /details matcher_ is initialized in this constructor
 */
tracker::tracker(matcher &matcher_) : matcher_(matcher_){};

/*!
 * \brief This function estimates the depth by triangulation.
 * 
 * \param[in] kpleft std::vector<cv::KeyPoint> type keypoints in left camera img
 * \param[in] kpRight keypoints in right camera img
 * \param[out] depth std::vector<double> type depth vector of corresponding keypoints
 * \details this fun is reserved for testing.
 */
void tracker::depthEstimation(const vector<KeyPoint> &kpLeft, const vector<KeyPoint> &kpRight, vector<double> &depth)
{
    depth.clear();
    int i = 0;
    double dx;
    double focalLenRec = 1741;
    double baseLine = 0.0428; //to be assigned
    while (i < kpLeft.size())
    {
        dx = kpLeft[i].pt.x - kpRight[i].pt.x;
        depth.push_back(focalLenRec * baseLine / dx);
        i++;
    };
    // cout << "Ave depth " << accumulate(depth.begin(), depth.end(), 0.0) / depth.size() << endl;
};

/*!
 * \brief This function reprojects 2d points to 3d in camera frame
 * 
 * \param[in] lastframe_ an instance of stereo
 * \param[in] currentFrame_ keypoints in right camera img
 */

void tracker::gen3DPoint(const stereoFrame &frame, vector<Point3d> &kp3d)
{
    kp3d.clear();
    vector<Point2d> kpl, kpr;
    Mat point3homo;
    int k = 0;

    for (int i = 0; i < frame.camRight.kpSelected.size(); i++)
    {
        kpl.push_back(frame.camLeft.kpSelected[i].pt);
        kpr.push_back(frame.camRight.kpSelected[i].pt);
    }

    if (frame.camRight.kpSelected.size() > 0)
    {
        // cout << frame.camLeft.recP << " " << frame.camRight.recP << endl;
        triangulatePoints(frame.camLeft.recP, frame.camRight.recP, kpl, kpr, point3homo);
    };

    for (int i = 0; i < kpl.size(); i++)
    {
        kp3d.push_back(Point3f(point3homo.at<double>(0, i) / point3homo.at<double>(3, i), point3homo.at<double>(1, i) / point3homo.at<double>(3, i), point3homo.at<double>(2, i) / point3homo.at<double>(3, i)));
    };
    // cout << kp3d << endl;
};

/*!
 * \brief This function find rotation and translation direction by essential mat decomposition
 * 
 * \param[in] kpLast keypoints in last frame 
 * \param[in] kpCurrent keypoints in current frame
 * \param[in] P projection matrix 
 */

void tracker::findRotation(const vector<KeyPoint> &kpLast, const vector<KeyPoint> &kpCurrent, const Mat &P)
{
    vector<Point2d> kpCoordRightLast, kpCoordRightCurrent;

    for (int i = 0; i < kpLast.size(); i++)
    {
        kpCoordRightLast.push_back(kpLast[i].pt);
        kpCoordRightCurrent.push_back(kpCurrent[i].pt);
        // cout << "Points collected " << kpCoordRightCurrent.size() << endl;
    };

    if (kpCoordRightLast.size() > 0)
    {
        double prob = 0.999;
        double threshold = 1.0;
        Mat mask, essentialMat, rotationMat1, rotationMat2;
        essentialMat = findEssentialMat(kpCoordRightLast, kpCoordRightCurrent, P.at<double>(0, 0), Point2d(P.at<double>(0, 2), P.at<double>(1, 2)), RANSAC, prob, threshold, mask);
        decomposeEssentialMat(essentialMat, rotationMat1, rotationMat2, translationDirection);

        Point3d transUnit;
        transUnit.x = translationDirection.at<double>(0);
        transUnit.y = translationDirection.at<double>(1);
        transUnit.z = translationDirection.at<double>(2);
        double ip;
        Point3d cp1, cp2;
        bool isInlier = false;
        int i = 0;
        while (isInlier == false)
        {
            if (mask.at<bool>(i) == 1)
            {
                isInlier = true;
            };
            i++;
        };

        cp1 = transUnit.cross(kp3dCurrent[i]);
        cp2 = transUnit.cross(this->MatPtMult(rotationMat1, kp3dLast[i]));
        ip = cp1.ddot(cp2);
        // cout << "InnerProduct " << ip << " " << ip2 << endl;
        if (ip > 0)
        {
            R = rotationMat1;
        }
        else
        {
            R = rotationMat2;
        };

        mask_ = mask;
        // cout << "transDir " << translationDirection << endl;
        // cout << rotationMat1.at<double>(0,0) << rotationMat2.at<double>(0,0) << endl;
        // if (rotationMat1.at<double>(0, 0) > rotationMat2.at<double>(0, 0))
        // {
        //     R = rotationMat1;
        // }
        // else
        // {
        //     R = rotationMat2;
        // };

        //find center
        Point3d centLast, centCurrent;
        int k = 0;
        for (int i = 0; i < kp3dLast.size(); i++)
        {
            if (mask.at<bool>(i) == 1)
            {
                centLast.x += kp3dLast[i].x;
                centLast.y += kp3dLast[i].y;
                centLast.z += kp3dLast[i].z;
                centCurrent.x += kp3dCurrent[i].x;
                centCurrent.y += kp3dCurrent[i].y;
                centCurrent.z += kp3dCurrent[i].z;
                k++;
            };
        };
        centLast = centLast / k;
        centCurrent = centCurrent / k;

        T = centCurrent - MatPtMult(R, centLast);
        // Point3d dirT = centCurrent - centLast;
        double theta = acos((trace(R).val[0] - 1) / 2) * 180 / 3.1415926535898;
        // cout << "theta" << theta << endl;
        if (theta < 1.5)
        {
            T = centCurrent - centLast;
        }

        if (theta < 0.3)
        {
            R = Mat::eye(3, 3, CV_64F);
        }
        // cout << "rotated trans: " << T << endl;
        // cout << "direct  trans: " << dirT << endl;
    };
};

/*!
 * \brief This function find translation by least square
 */
void tracker::findTranslation()
{
    if (kp3dLast.size() > 0)
    {
        Point3d transUnit;
        transUnit.x = translationDirection.at<double>(0);
        transUnit.y = translationDirection.at<double>(1);
        transUnit.z = translationDirection.at<double>(2);
        Point3d kpTransAve;
        kpTranslation.clear();
        int k = 0;
        for (int i = 0; i < kp3dCurrent.size(); i++)
        {
            kpTranslation.push_back(kp3dCurrent[i] - this->MatPtMult(R, kp3dLast[i]));
            // kpTranslation.push_back(kp3dCurrent[i] - kp3dLast[i]);
        };

        // doing least squares
        double tempNumerator = 0;
        double tempDenominator = 0;
        for (int i = 0; i < kpTranslation.size(); i++)
        {
            tempNumerator += kpTranslation[i].ddot(kpTranslation[i]);
            tempDenominator += kpTranslation[i].ddot(transUnit);
        };
        double fittedTransAbs = tempNumerator / tempDenominator;
        T = fittedTransAbs * transUnit;
        cout << "bestTranslation: " << T << endl;
    }
};

void tracker::findRTQtn()
{
    if (kp3dLast.size() > 0)
    {
        //find center
        Point3d centLast, centCurrent;
        for (int i = 0; i < kp3dLast.size(); i++)
        {
            centLast.x += kp3dLast[i].x / kp3dLast.size();
            centLast.y += kp3dLast[i].y / kp3dLast.size();
            centLast.z += kp3dLast[i].z / kp3dLast.size();
            centCurrent.x += kp3dCurrent[i].x / kp3dCurrent.size();
            centCurrent.y += kp3dCurrent[i].y / kp3dCurrent.size();
            centCurrent.z += kp3dCurrent[i].z / kp3dCurrent.size();
        };
        //center point clouds
        vector<Point3d> kp3dLastCtd, kp3dCurrentCtd;
        for (int i = 0; i < kp3dLast.size(); i++)
        {
            kp3dLastCtd.push_back(kp3dLast[i] - centLast);
            kp3dCurrentCtd.push_back(kp3dCurrent[i] - centCurrent);
        };

        Mat kpLastMat = Mat(kp3dLastCtd.size(), 3, CV_64F, kp3dLastCtd.data());
        Mat kpCurrentMat = Mat(kp3dCurrentCtd.size(), 3, CV_64F, kp3dCurrentCtd.data());

        Mat kpLastMatTransp;
        transpose(kpLastMat, kpLastMatTransp);
        // cout << kpCurrentMatTransp.size() << kpLastMat.size() << endl;

        Mat M = kpLastMatTransp * kpCurrentMat;
        // cout << M.size() << endl;

        double Sxx = M.at<double>(0, 0);
        double Syx = M.at<double>(1, 0);
        double Szx = M.at<double>(2, 0);
        double Sxy = M.at<double>(0, 1);
        double Syy = M.at<double>(1, 1);
        double Szy = M.at<double>(2, 1);
        double Sxz = M.at<double>(0, 2);
        double Syz = M.at<double>(1, 2);
        double Szz = M.at<double>(2, 2);
        Mat N = (Mat_<double>(4, 4) << (Sxx + Syy + Szz), (Syz - Szy), (Szx - Sxz), (Sxy - Syx), (Syz - Szy), (Sxx - Syy - Szz), (Sxy + Syx), (Szx + Sxz), (Szx - Sxz), (Sxy + Syx), (-Sxx + Syy - Szz), (Syz + Szy), (Sxy - Syx), (Szx + Sxz), (Syz + Szy), (-Sxx - Syy + Szz));
        Mat eigenvalues, eigenvectors;
        eigen(N, eigenvalues, eigenvectors);
        double minev, maxev;
        Point minLoc, maxLoc;

        minMaxLoc(eigenvalues, &minev, &maxev, &minLoc, &maxLoc);

        double eval = eigenvalues.at<double>(maxLoc);
        Mat evec = eigenvectors.row(maxLoc.y);

        minMaxLoc(abs(evec), &minev, &maxev, &minLoc, &maxLoc);
        int sgn = evec.at<double>(maxLoc) / abs(evec.at<double>(maxLoc));

        double q0 = sgn * evec.at<double>(0);
        double qx = sgn * evec.at<double>(1);
        double qy = sgn * evec.at<double>(2);
        double qz = sgn * evec.at<double>(3);
        Mat v = evec(Range(0, 1), Range(1, 4));

        Mat z = (Mat_<double>(3, 3) << q0, -qz, qy, qz, q0, -qx, -qy, qx, q0);

        Mat vTrans;
        transpose(v, vTrans);

        R = vTrans * v + z * z;
        T = centCurrent - MatPtMult(R, centLast);
        // T = centCurrent - centLast;
        cout << R << "\n"
             << T << endl;
    };
};

void tracker::findRTSVD()
{
    if (kp3dLast.size() > 0)
    {
        //find center
        Point3d centLast, centCurrent;
        for (int i = 0; i < kp3dLast.size(); i++)
        {
            centLast.x += kp3dLast[i].x / kp3dLast.size();
            centLast.y += kp3dLast[i].y / kp3dLast.size();
            centLast.z += kp3dLast[i].z / kp3dLast.size();
            centCurrent.x += kp3dCurrent[i].x / kp3dCurrent.size();
            centCurrent.y += kp3dCurrent[i].y / kp3dCurrent.size();
            centCurrent.z += kp3dCurrent[i].z / kp3dCurrent.size();
        };
        //center point clouds
        vector<Point3d> kp3dLastCtd, kp3dCurrentCtd;
        for (int i = 0; i < kp3dLast.size(); i++)
        {
            kp3dLastCtd.push_back(kp3dLast[i] - centLast);
            kp3dCurrentCtd.push_back(kp3dCurrent[i] - centCurrent);
        };

        Mat kpLastMat = Mat(kp3dLastCtd.size(), 3, CV_64F, kp3dLastCtd.data());
        Mat kpCurrentMat = Mat(kp3dCurrentCtd.size(), 3, CV_64F, kp3dCurrentCtd.data());

        Mat kpCurrentMatTransp;
        transpose(kpCurrentMat, kpCurrentMatTransp);
        // cout << kpCurrentMatTransp.size() << kpLastMat.size() << endl;
        Mat M = kpCurrentMatTransp * kpLastMat;
        Mat U, W, V;
        SVD::compute(M, W, U, V);
        R = U * V;
        // cout << tempR << endl;
        T = centCurrent - MatPtMult(R, centLast);
    };
};

void tracker::runTracking(stereoFrame &currentFrame_, stereoFrame &lastFrame_)
{

    vector<DMatch> rawMatchesLastCurrent, rawMatchesCurrentLast, uniqueMatchesLastCurrent;

    vector<double> depth;

    matcher_.matchDescriptors(lastFrame_.camRight.desSelected, currentFrame_.camRight.desSelected, rawMatchesLastCurrent);

    if (rawMatchesLastCurrent.size() > 0)
    {
        matcher_.matchDescriptors(currentFrame_.camRight.desSelected, lastFrame_.camRight.desSelected, rawMatchesCurrentLast);
    };
    // cout << rawMatchesLastCurrent.size() << " " << rawMatchesCurrentLast.size() << endl;

    matcher_.biDirecRejector(rawMatchesLastCurrent, rawMatchesCurrentLast, uniqueMatchesLastCurrent);
    matcher_.matchFilter(uniqueMatchesLastCurrent, 0.4, matchesLastCurrent);

    // cout << goodMatchesLastCurrent.size() << endl;

    matcher_.updateMatchPoints2(lastFrame_.camLeft, lastFrame_.camRight, currentFrame_.camLeft, currentFrame_.camRight, lastFrame_.matches, currentFrame_.matches, matchesLastCurrent);

    this->depthEstimation(currentFrame_.camLeft.kpSelected, currentFrame_.camRight.kpSelected, depth);

    this->gen3DPoint(lastFrame_, kp3dLast);
    this->gen3DPoint(currentFrame_, kp3dCurrent);
    this->findRotation(lastFrame_.camRight.kpSelected, currentFrame_.camRight.kpSelected, currentFrame_.camLeft.recP);

    // this->findRTQtn();
    // this->findRTSVD();
    // this->findTranslation();
};

/*! 
 * \brief This func multiplies a cv::Mat type 3*3 matrix and a 3D point in Point3_ structure
 * 
 * \param[in] M The rotation matrix
 * \param[in] p Point to be rotated
 */

Point3d tracker::MatPtMult(const Mat M, const Point3d &p)
{
    Mat_<double> src(3, 1);
    src.at<double>(0, 0) = p.x;
    src.at<double>(1, 0) = p.y;
    src.at<double>(2, 0) = p.z;

    // //USE MATRIX ALGEBRA
    M.convertTo(M, CV_64FC1);
    src.convertTo(src, CV_64FC1);
    Mat dst = M * src;

    return Point3d(dst.at<double>(0, 0), dst.at<double>(1, 0), dst.at<double>(2, 0));
};