#ifndef CSTEREOMATCH_H
#define CSTEREOMATCH_H
#include "cparamcalib.h"
#include "opencv.hpp"
#include "qdebug.h"
#include "legacy/legacy.hpp"
#include "cvaux.hpp"
class CStereoMatch
{
public:
    CStereoMatch();
    ~CStereoMatch();
    void CalcDispa(Mat & left_image, Mat & right_image,Mat & dispa_image,int flag);
    void DispaColor(Mat& m_depth8, Mat& m_show_dispa);
public:
    CParamCalib m_param;
    StereoBM BM;
    StereoSGBM SGBM;
    StereoVar var;
    Mat m_depth;//深度图
    Mat XYZ;
    vector<Mat> channnels;
    int m_color_value;
    CvStereoGCState *GCState;
};

#endif // CSTEREOMATCH_H
