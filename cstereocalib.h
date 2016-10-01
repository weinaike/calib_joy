#ifndef CSTEREOCALIB_H
#define CSTEREOCALIB_H
#include "qstringlist.h"
#include "cparamcalib.h"
#include "qimage.h"
#include "qimagemat.h"

using namespace cv;
class CStereoCalib
{
public:
    CStereoCalib();
    double CalibStart();
    QImage getcorners(QString &,vector< vector<Point3f> >&,vector< vector<Point2f> > &);


public:
    Size m_imagesize;
    QStringList m_fleftlist,m_frightlist;
    int num_used;//棋盘图片数
    Size m_boardsize; //内角点数量
    float m_squarelength;//棋盘正方形尺寸mm
    double m_err1,m_err2,m_err;//标定误差系数
    vector< vector<Point3f> > m_objectPointVect1;
    vector< vector<Point2f> > m_imagePointVect1;
    vector< vector<Point3f> > m_objectPointVect2;
    vector< vector<Point2f> > m_imagePointVect2;
    vector<Mat> m_rvec1;//旋转矢量
    vector<Mat> m_tvec1;//平移矢量
    vector<Mat> m_rvec2;//旋转矢量
    vector<Mat> m_tvec2;//平移矢量
    int flag;
    CParamCalib m_param;
    bool flag_stereo;
};

#endif // CSTEREOCALIB_H
