#include "../include/matcher.hpp"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

/*!
 * \brief matcher constructor
 * 
 * \param[in] method matching method :`BruteForce` (it uses L2 ), `BruteForce-L1`, `BruteForce-Hamming`, 
 * `BruteForce-Hamming(2)`, `FlannBased`
 */
matcher::matcher(const string method)
{
    matchMethod = method;
    thismatcher = DescriptorMatcher::create(matchMethod);
};

/*!
 * \brief this function matches descriptors
 * 
 * \param[in] descriptors_1 descriptors to be matched
 * \param[in] descriptors_2 same to descriptors_1
 * \param[out] thismatch matching indexes
 */
void matcher::matchDescriptors(const Mat &descriptors_1, const Mat &descriptors_2, vector<DMatch> &thismatch)
{
    thismatcher->match(descriptors_1, descriptors_2, thismatch);
    // cout << "Num of raw matches" << thismatch.size() << endl;
};

/*!
 * \brief this function filters matching results by matching distance
 * 
 * \param[in] rawMatches raw matches to be filtered
 * \param[in] threshold threshold ranges from 0 to 1. higher threshold tolorant larger matching distance.
 * fewer matches of lower matching distance will be returned with lower threshold. 
 * \param[out] goodMatches filtered matches
 * \details this is match filter working on the matching distance. The largest and smallest distance will
 * first be find out. Than the threshold passed in will generate a distance which is 
 * (smallest distance)+threshold*(largest distance - smallest distance). The matches larger than this distance will be filtered.
 */
void matcher::matchFilter(const vector<DMatch> &rawMatches, const double threshold, vector<DMatch> &goodMatches)
{
    goodMatches.clear();
    double min_dist = 10000, max_dist = 0;
    // find max and min distance in discriptor matches
    for (int i = 0; i < rawMatches.size(); i++)
    {
        double dist = rawMatches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    };
    // cout << "Max dist : " << max_dist << endl;
    // cout << "Min dist : " << min_dist << endl;

    //reject descriptor distane larger than n times of min distance
    for (int i = 0; i < rawMatches.size(); i++)
    {
        if (rawMatches[i].distance <= (min_dist + threshold * (max_dist - min_dist)))
        {
            goodMatches.push_back(rawMatches[i]);
        };
    };
    // cout << "Num of good matches " << goodMatches.size() << endl;
};

/*!
 * \brief this function filters matching results by epipolar constriants
 * 
 * \param[in] goodMatches matches to be filtered
 * \param[in] camLeft left camera
 * \param[in] camRight right camera
 * \param[out] horizontalMatches filtered matches
 */
void matcher::epipolarRejector(const vector<DMatch> &goodMatches, camera &camLeft, camera &camRight, vector<DMatch> &horizontalMatches)
{
    horizontalMatches.clear();
    for (int i = 0; i < goodMatches.size(); i++)
    {
        if (abs(camLeft.kp[goodMatches[i].queryIdx].pt.y - camRight.kp[goodMatches[i].trainIdx].pt.y) <= 4)
        {
            horizontalMatches.push_back(goodMatches[i]);
        }
    }
    // cout << "Num of horizontal matches " << horizontalMatches.size() << endl;
};

/*!
 * \brief this function filters matching results by bi-directional check
 * 
 * \param[in] rawMatches1 matches to be filtered
 * \param[in] rawMatches2 inverse matching of rawMatches1
 * \param[out] filteredMatches filtered matches
 */
void matcher::biDirecRejector(const vector<DMatch> &rawMatches1, const vector<DMatch> &rawMatches2, vector<DMatch> &filteredMatches)
{
    filteredMatches.clear();
    int inlierCounter = 0;
    int outlierCounter = 0;
    // cout << rawMatches1.size() << " " << rawMatches2.size() << endl;
    for (int i = 0; i < rawMatches1.size(); i++)
    {
        for (int j = 0; j < rawMatches2.size(); j++)
        {
            if (rawMatches2[j].trainIdx == rawMatches1[i].queryIdx)
            {
                if (rawMatches2[j].queryIdx == rawMatches1[i].trainIdx)
                {
                    filteredMatches.push_back(rawMatches1[i]);
                    inlierCounter++;
                }
                else
                {
                    outlierCounter++;
                };
            };
        };
    };
    // cout << inlierCounter << " " << outlierCounter << endl;
};

void matcher::updateMatchPoints(camera &camLeft, camera &camRight, const vector<DMatch> &matches)
{
    // clear
    camLeft.desSelected.release();
    camRight.desSelected.release();

    for (int i = 0; i < matches.size(); i++)
    {
        // select good matches in kp and descriptors
        camLeft.desSelected.push_back(camLeft.descriptors.row(matches[i].queryIdx));
        camRight.desSelected.push_back(camRight.descriptors.row(matches[i].trainIdx));
    }
    // cout << kpLeftCurrentSelected.size() << " " << kpLeftCurrentSelected.size() << endl;
};

void matcher::updateMatchPoints2(camera &camLeftLast, camera &camRightLast, camera &camLeftCurrent, camera &camRightCurrent, const vector<DMatch> &matches, const vector<DMatch> &matches2, const vector<DMatch> &matches3)
{
    // clear
    camLeftLast.kpSelected.clear();
    camRightLast.kpSelected.clear();
    camLeftCurrent.kpSelected.clear();
    camRightCurrent.kpSelected.clear();

    for (int i = 0; i < matches3.size(); i++)
    {
        // select good matches in kp and descriptors
        camLeftLast.kpSelected.push_back(camLeftLast.kp[matches[matches3[i].queryIdx].queryIdx]);
        camRightLast.kpSelected.push_back(camRightLast.kp[matches[matches3[i].queryIdx].trainIdx]);
        camLeftCurrent.kpSelected.push_back(camLeftCurrent.kp[matches2[matches3[i].trainIdx].queryIdx]);
        camRightCurrent.kpSelected.push_back(camRightCurrent.kp[matches2[matches3[i].trainIdx].trainIdx]);
    }
};