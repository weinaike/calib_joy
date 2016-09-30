#include "cstereocalib.h"

CStereoCalib::CStereoCalib()
{

}


double CStereoCalib::CalibStart()
{

    Mat cameraMat1,DistMat1;//内部参数，外部参数
    m_err1=calibrateCamera(m_objectPointVect1,m_imagePointVect1,m_imagesize,cameraMat1,DistMat1,m_rvec1,m_tvec1,
                                CV_CALIB_FIX_K3);
    qDebug()<<"err1:"<<sqrt(m_err1/81);
    Mat cameraMat2,DistMat2;//内部参数，外部参数
    m_err2=calibrateCamera(m_objectPointVect2,m_imagePointVect2,m_imagesize,cameraMat2,DistMat2,m_rvec2,m_tvec2,
                                CV_CALIB_FIX_K3);
    qDebug()<<"err2:"<<sqrt(m_err2/81);
    Mat R,T,E,F,Rout;

    m_err=stereoCalibrate(m_objectPointVect1,m_imagePointVect1,m_imagePointVect2,cameraMat1,DistMat1,cameraMat2,DistMat2,m_imagesize,R,T,E,F,
                                   TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                   CV_CALIB_FIX_INTRINSIC);
    Rodrigues(R,Rout);
    qDebug()<<"cameraMat1"<<cameraMat1.at<double>(0,0);
    qDebug()<<"cameraMat2"<<cameraMat2.at<double>(0,0);
    qDebug()<<"T:"<<T.at<double>(0,0);
    Mat R1,R2,P1,P2,Q;
    Rect roi1,roi2;
    stereoRectify(cameraMat1,DistMat1,cameraMat2,DistMat2,m_imagesize,R,T,R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,-1,Size(),&roi1,&roi2);
    Mat map1x,map1y,map2x,map2y;
    initUndistortRectifyMap(cameraMat1,DistMat1,R1,P1,m_imagesize,CV_32FC1,map1x,map1y);
    initUndistortRectifyMap(cameraMat2,DistMat2,R2,P2,m_imagesize,CV_32FC1,map2x,map2y);
    m_param.m_CameraMat1=cameraMat1;
    m_param.m_CameraMat2=cameraMat2;
    m_param.m_DistMat1=DistMat1;
    m_param.m_DistMat2=DistMat2;
    m_param.m_map1x=map1x;
    m_param.m_map1y=map1y;
    m_param.m_map2x=map2x;
    m_param.m_map2y=map2y;
    m_param.m_P1=P1;
    m_param.m_P2=P2;
    m_param.m_Q=Q;
    m_param.m_R=R;
    m_param.m_R1=R1;
    m_param.m_R2=R2;
    m_param.m_roi1=roi1;
    m_param.m_roi2=roi2;
    m_param.m_T=T;
    m_param.SaveParam();

    return sqrt(m_err/m_boardsize.width/m_boardsize.height);
}
QImage CStereoCalib::getcorners(QString & filename,vector< vector<Point3f> > &m_objectPointVect,vector< vector<Point2f> > &m_imagePointVect)
{

    Mat src=imread(filename.toStdString(),1);
    Mat src_gray;
    vector<Point3f> objectPoint;
    vector<Point2f> imagePoint;
    m_imagesize=src.size();
    bool found;
    cvtColor(src,src_gray,CV_RGB2GRAY);
    found=findChessboardCorners(src_gray,m_boardsize,imagePoint, CV_CALIB_CB_ADAPTIVE_THRESH);
    if(found){
        //图像空间的像点
        cornerSubPix(src_gray,Mat(imagePoint),Size((m_boardsize.width-1)/2,(m_boardsize.height-1)/2),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
        drawChessboardCorners(src_gray,m_boardsize,Mat(imagePoint),found);
        m_imagePointVect.push_back(imagePoint);
        //世界空间的物点
        for(int i=0;i<m_boardsize.width;i++)
            for(int j=0;j<m_boardsize.height;j++){
                objectPoint.push_back(Point3f(j*m_squarelength,i*m_squarelength,0));
            }
        m_objectPointVect.push_back(objectPoint);
    }
    else
    {
        qDebug()<<"error: corners is not complete";
    }

    QImage image_out=Mat2QImage(src_gray);
    return(image_out);
}
