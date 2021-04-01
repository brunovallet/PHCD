#pragma once

// OpenCv utils
// Created 31/07/2012 by Bruno Vallet

#include "opencv/cv.h"
#include <string>

std::string CvTypename(int type);

std::string Info(const cv::Mat & img);

bool CheckSameSize(const cv::Mat & img1, const cv::Mat & img2);

template<class T> cv::Mat Tab2Mat(T * tab, int nl, int nc)
{
    // todo
}

template<class T> T * Mat2Tab(cv::Mat & img)
{
    T * tab = new T[img.rows*img.cols];
    img.data;
}

/// read a binary file of floats
cv::Mat ReadBinFloat(std::string filename, int width=0, int n_channel=1);

/// read a with open CV or binary file of floats
cv::Mat ReadImgOrBin(std::string filename, int width=0);

/// single channel only
void SaveBin(const std::string & filename, const cv::Mat & img);

/// bin for single channel float, else with OpenCv
void SaveImgOrBin(const std::string & filename, const cv::Mat & img);

/// split: receives a char delimiter; returns a vector of strings
/// By default ignores repeated delimiters, unless argument rep == 1.
std::vector<std::string> split(std::string s, char delim, int rep=0);
