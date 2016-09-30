#include "cstereomatch.h"

CStereoMatch::CStereoMatch()
{
    GCState= cvCreateStereoGCState(16,2);
}

CStereoMatch::~CStereoMatch()
{
    cvReleaseStereoGCState(&GCState);
}

void CStereoMatch::CalcDispa(Mat & m_src_left, Mat & m_src_right, Mat & m_show_dispa, int flag)
{
    Mat dst1,dst2,m_dispa,m_depth8;

    remap(m_src_left,m_src_left,m_param.m_map1x,m_param.m_map1y,INTER_NEAREST);
    remap(m_src_right,m_src_right,m_param.m_map2x,m_param.m_map2y,INTER_NEAREST);

    cvtColor(m_src_left,dst1,CV_RGB2GRAY);
    cvtColor(m_src_right,dst2,CV_RGB2GRAY);

    if(flag==0)
    {
        cvtColor(m_src_left,dst1,CV_RGB2GRAY);
        cvtColor(m_src_right,dst2,CV_RGB2GRAY);
        BM(dst1,dst2,m_dispa);//m_dispa 16bit
    }
    if(flag==1)
    {
        dst1=m_src_left;
        dst2=m_src_right;
        SGBM(dst1,dst2,m_dispa);//m_dispa 16bit
    }
    if(flag==2)
    {
        dst1=m_src_left;
        dst2=m_src_right;
        var(dst1,dst2,m_dispa);//m_dispa 8bit
    }
    if(flag==3)
    {
        IplImage left=IplImage(dst1);
        IplImage right= IplImage(dst2);
        CvMat*  left_disp=cvCreateMat(dst1.rows,dst1.cols,CV_16S);
        CvMat*  right_disp=cvCreateMat(dst1.rows,dst1.cols,CV_16S);
        cvFindStereoCorrespondenceGC(&left,&right,left_disp,right_disp,GCState,0);
        CvMat* disparity_left_visual = cvCreateMat( dst1.rows, dst1.cols, CV_8U );
        cvConvertScale( left_disp, disparity_left_visual, -16 );
        m_dispa=Mat(disparity_left_visual);
    }
    reprojectImageTo3D(m_dispa,XYZ,m_param.m_Q,true,CV_32F);

    split(XYZ,channnels);
    m_depth=channnels.at(2);

    //if(flag==0) m_depth.convertTo(m_depth8, CV_8U,255.0/(BM.state->numberOfDisparities*16));
    if(flag==0) m_depth.convertTo(m_depth8, CV_8U,255.0/m_color_value);
    if(flag==1) m_depth.convertTo(m_depth8, CV_8U,255.0/m_color_value);
    //if(flag==1) m_depth.convertTo(m_depth8, CV_8U,255.0/(SGBM.numberOfDisparities*16));
    if(flag==2)
    {   m_depth=m_depth/4;
        m_depth.convertTo(m_depth8, CV_8U,255.0/m_color_value);
    }
    if(flag==3)
    {   m_depth=m_depth/4;
        m_depth.convertTo(m_depth8, CV_8U,255.0/m_color_value);
    }


    double min,max;
    minMaxLoc(m_depth,&min,&max);
    qDebug()<<min<<" "<<max<<" "<<m_depth.at<float>(240,320)<<" "<<m_depth8.at<uchar>(240,320);

    DispaColor(m_depth8,m_show_dispa);

}


void CStereoMatch::DispaColor(Mat& m_depth8, Mat& m_show_dispa)
{

    for (int y=0;y<m_depth8.rows;y++)
    {
        for (int x=0;x<m_depth8.cols;x++)
        {

            uchar tmp2 = m_depth8.at<uchar>(y,x);
            if (tmp2 <= 51)
            {
                m_show_dispa.at<Vec3b>(y,x)[0]= 255;
                m_show_dispa.at<Vec3b>(y,x)[1]= tmp2*5;
                m_show_dispa.at<Vec3b>(y,x)[2]= 0;
            }
            else if (tmp2 <= 102)
            {
                tmp2-=51;
                m_show_dispa.at<Vec3b>(y,x)[0]= 255-tmp2*5;
                m_show_dispa.at<Vec3b>(y,x)[1]= 255;
                m_show_dispa.at<Vec3b>(y,x)[2]= 0;

            }
            else if (tmp2 <= 153)
            {
                tmp2-=102;
                m_show_dispa.at<Vec3b>(y,x)[0]= 0;
                m_show_dispa.at<Vec3b>(y,x)[1]= 255;
                m_show_dispa.at<Vec3b>(y,x)[2]= tmp2*5;

            }
            else if (tmp2 <= 204)
            {
                tmp2-=153;
                m_show_dispa.at<Vec3b>(y,x)[0]= 0;
                m_show_dispa.at<Vec3b>(y,x)[1]= 255-uchar(128.0*tmp2/51.0+0.5);
                m_show_dispa.at<Vec3b>(y,x)[2]= 255;

            }
            else
            {
                tmp2-=204;
                m_show_dispa.at<Vec3b>(y,x)[0]= 0;
                m_show_dispa.at<Vec3b>(y,x)[1]= 127-uchar(127.0*tmp2/51.0+0.5);
                m_show_dispa.at<Vec3b>(y,x)[2]= 255;

            }
        }
    }

}
