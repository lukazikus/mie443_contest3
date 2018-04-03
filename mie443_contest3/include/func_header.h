#ifndef FUNC_HEADER_H
#define FUNC_HEADER_H

#include <nav_header.h>
#include <imageTransporter.hpp>

using namespace std;

bool init(vector<vector<float> >& coord, std::vector<cv::Mat>& imgs_track);

//int findPic(imageTransporter &imgTransport, vector<cv::Mat> &imgs_track, int iteration); // For actual function
int findPic(imageTransporter &imgTransport, cv::Mat &imgs_track, int iteration); // modified actual function
//int findPic(cv::Mat &img_object, cv::Mat &imgs_track, int iteration);
double feature2D_homography(cv::Mat img_object, cv::Mat img_scene );

#endif
