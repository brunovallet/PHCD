
// OpenCv utils
// Created 31/07/2012 by Bruno Vallet

#include "CvUtils.h"
#include "opencv/highgui.h"
#include <iostream>

using namespace std;

// utilities
string CvTypename(int type)
{
    switch(type)
    {
    case CV_8U: return "CV_8U";
    case CV_8S: return "CV_8S";
    case CV_16U: return"CV_16U";
    case CV_16S: return"CV_16S";
    case CV_32S: return"CV_32S";
    case CV_32F: return"CV_32F";
    case CV_64F: return"CV_64F";

    case CV_8UC2: return "CV_8UC2";
    case CV_8SC2: return "CV_8SC2";
    case CV_16UC2: return"CV_16UC2";
    case CV_16SC2: return"CV_16SC2";
    case CV_32SC2: return"CV_32SC2";
    case CV_32FC2: return"CV_32FC2";
    case CV_64FC2: return"CV_64FC2";

    case CV_8UC3: return "CV_8UC3";
    case CV_8SC3: return "CV_8SC3";
    case CV_16UC3: return"CV_16UC3";
    case CV_16SC3: return"CV_16SC3";
    case CV_32SC3: return"CV_32SC3";
    case CV_32FC3: return"CV_32FC3";
    case CV_64FC3: return"CV_64FC3";

    case CV_8UC4: return "CV_8UC4";
    case CV_8SC4: return "CV_8SC4";
    case CV_16UC4: return"CV_16UC4";
    case CV_16SC4: return"CV_16SC4";
    case CV_32SC4: return"CV_32SC4";
    case CV_32FC4: return"CV_32FC4";
    case CV_64FC4: return"CV_64FC4";
    }
    return "unknown";
}

string Info(const cv::Mat & img)
{
    ostringstream oss;
    oss << img.cols << "x" << img.rows << "/" << CvTypename(img.type()) << ", " <<
           1.e-6*img.rows*img.cols << "Mpx";
    return oss.str();
}
bool CheckSameSize(const cv::Mat & img1, const cv::Mat & img2)
{
    if (img1.rows != img2.rows) {cout << "Wrong height: " << img1.rows << "!=" << img2.rows << endl; return false;}
    if (img1.cols != img2.cols) {cout << "Wrong width: " << img1.cols << "!=" << img2.cols << endl; return false;}
    return true;
}

/// read a bin image where width is given
cv::Mat ReadBinFloat(string filename, int width, int n_channel)
{
    cv::Mat img;
    if(n_channel<1 || n_channel>4)
    {
        cout << "ERROR: called ReadBin(...) with " << n_channel << " channels";
        return img;
    }
    FILE * pFile = fopen ( filename.c_str() , "rb" );
    if (pFile==NULL) {cout << __FUNCTION__ << " Cannot open file " << filename << endl; return img;}

    // obtain file size:
    fseek (pFile , 0 , SEEK_END);
    long lSize = ftell (pFile);
    rewind (pFile);
    unsigned int n_float = lSize/sizeof(float), n_pix = n_float/n_channel, height;
    if(width == 0) {width = sqrt(n_pix); height = width;}
    else {height = n_pix/width;}
    if(width*height != n_pix) {
        cout << "Error: number of pixels " << n_pix << " inconsistent with size "
             << width << "x" << height << endl;
        return img;
    }
    //cout << "--Reading bin as image of size " << width << "x" << height << endl;

    // allocate memory to contain the whole file:
    float * buffer = (float*) malloc(n_float*sizeof(float));
    if (buffer == NULL) {fputs ("Memory error\n",stderr); exit (5);}

    // copy the file into the buffer:
    size_t result = fread (buffer,sizeof(float),n_float,pFile);
    if (result != n_float) {fputs ("Reading error\n",stderr); exit (6);}
    fclose (pFile);
    int type = CV_32FC1;
    switch(n_channel)
    {
    case 2: type = CV_32FC2; break;
    case 3: type = CV_32FC3; break;
    case 4: type = CV_32FC4; break;
    }
    img = cv::Mat(width, height, type, buffer, cv::Mat::AUTO_STEP);
    //cout << "buffer=" << buffer << " " << "img.data=" << (float*)img.data << endl;

    if(!img.data) {cout << "Cannot create OpenCV Mat from bin data" << endl;}
    return img;
}

cv::Mat ReadImgOrBin(string filename, int width)
{
    cv::Mat img = cv::imread(filename, -1);
    if(!img.data)
    {
        //cout << "Cannot read " << filename << " with openCV, trying .bin mode" << endl;
        return ReadBinFloat(filename, width);
    }
    return img;
}

void SaveBin(const string & filename, const cv::Mat & img)
{
    FILE * pFile = fopen (filename.c_str(), "wb");
    fwrite (img.data, sizeof(float), img.cols*img.rows, pFile);
    fclose (pFile);
}

void SaveImgOrBin(const string & filename, const cv::Mat & img)
{
    if(img.type() == CV_32F) SaveBin(filename, img);
    else cv::imwrite(filename, img);
}

vector<string> split(string s, char delim, int rep)
{
    vector<string> flds;
    if (!flds.empty()) flds.clear();  // empty vector if necessary
    string work = s.data();
    string buf = "";
    int i = 0;
    while (i < work.length()) {
        if (work[i] != delim)
            buf += work[i];
        else if (rep == 1) {
            flds.push_back(buf);
            buf = "";
        } else if (buf.length() > 0) {
            flds.push_back(buf);
            buf = "";
        }
        i++;
    }
    if (!buf.empty())
        flds.push_back(buf);
    return flds;
}
