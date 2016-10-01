#ifndef CPARAMCALIB_H
#define CPARAMCALIB_H
#include "opencv.hpp"
#include <string>
using namespace cv;

class CParamCalib
{
public:
    CParamCalib();
    void GetParam();
    void SaveParam();


public:
    Mat m_CameraMat1;
    Mat m_CameraMat2;
    Mat m_DistMat1;
    Mat m_DistMat2;
    Mat m_R1,m_R2,m_P1,m_P2,m_Q,m_R,m_T;
    Rect m_roi1,m_roi2;
    Mat m_map1x,m_map1y,m_map2x,m_map2y;
};

#endif // CPARAMCALIB_H
