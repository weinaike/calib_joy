#include "dialog.h"
#include "ui_dialog.h"
#include "qfiledialog.h"
#include "opencv.hpp"
#include "qdebug.h"
using namespace cv;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{

    ui->setupUi(this);

    m_pStereoCalib=new CStereoCalib;
    m_pStereoMatch=new CStereoMatch;
    ui->pushButton_CloseCam->setDisabled(true);
    ui->pushButton_DispStart->setDisabled(true);
    ui->pushButton_DispStop->setDisabled(true);
    ui->pushButton_Calib->setDisabled(false);
    ui->pushButton_OpenCam->setEnabled(true);
    num_camera = 0;
}

Dialog::~Dialog()
{
    delete ui;
    delete m_pStereoCalib;
    delete m_pStereoMatch;
}

void Dialog::on_lineEdit_px_editingFinished()
{

}


void Dialog::onTimer(){    

    QImage imagescaled=m_image_left.scaled(320,240,Qt::KeepAspectRatio);
    ui->label_left->setPixmap(QPixmap::fromImage(imagescaled));
    imagescaled=m_image_right.scaled(320,240,Qt::KeepAspectRatio);
    ui->label_right->setPixmap(QPixmap::fromImage(imagescaled));

}

void Dialog::onTimer_cam()
{
    if(ui->checkBox_rectify->isChecked())
    {
        m_cap_left>>m_src_left;
        m_cap_right>>m_src_right;

        if(!m_pStereoCalib->m_param.m_map1x.empty())
        {
            remap(m_src_left,m_src_left,m_pStereoCalib->m_param.m_map1x,m_pStereoCalib->m_param.m_map1y,INTER_NEAREST);
            remap(m_src_right,m_src_right,m_pStereoCalib->m_param.m_map2x,m_pStereoCalib->m_param.m_map2y,INTER_NEAREST);
        }
        else
        {
            QMessageBox::about(this,"error","no calibration");
            ui->checkBox_rectify->setChecked(false);
        }

        m_image_left=Mat2QImage(m_src_left);
        m_image_right=Mat2QImage(m_src_right);
        QImage imagescaled=m_image_left.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_left->setPixmap(QPixmap::fromImage(imagescaled));
        imagescaled=m_image_right.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_right->setPixmap(QPixmap::fromImage(imagescaled));

    }
    else
    {
        m_cap_left>>m_src_left;
        m_cap_right>>m_src_right;
        //GaussianBlur(m_src_left,m_src_left,Size(9,9),0,0);
        //blur(m_src_right,m_src_right,Size(5,5));
        m_image_left=Mat2QImage(m_src_left);
        m_image_right=Mat2QImage(m_src_right);
        QImage imagescaled=m_image_left.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_left->setPixmap(QPixmap::fromImage(imagescaled));
        imagescaled=m_image_right.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_right->setPixmap(QPixmap::fromImage(imagescaled));
    }

}


void Dialog::on_pushButton_CloseCam_clicked()
{
    m_timer.stop();
    m_cap_left.release();
    m_cap_right.release();

    ui->pushButton_CloseCam->setDisabled(true);
    ui->pushButton_DispStart->setDisabled(true);
    ui->pushButton_DispStop->setDisabled(true);
    ui->pushButton_OpenCam->setEnabled(true);

}

void Dialog::on_pushButton_OpenCam_clicked()
{
    m_timer.stop();
    m_cap_left.open(0);
    if(m_cap_left.isOpened())
        num_camera++;
    m_cap_right.open(1);
    if(m_cap_right.isOpened())
        num_camera++;
    m_cap_left.set(CV_CAP_PROP_FRAME_HEIGHT,640);
    m_cap_left.set(CV_CAP_PROP_FRAME_WIDTH,480);
    m_cap_right.set(CV_CAP_PROP_FRAME_HEIGHT,640);
    m_cap_right.set(CV_CAP_PROP_FRAME_WIDTH,480);
    connect(&m_timer,SIGNAL(timeout()),this,SLOT(onTimer_cam()));
    m_timer.start(200);

    ui->pushButton_OpenCam->setDisabled(true);
    ui->pushButton_CloseCam->setEnabled(true);
    ui->pushButton_DispStart->setEnabled(true);

}

void Dialog::on_pushButton_Calib_clicked()
{
    //导入界面参数
    connect(&m_timer,SIGNAL(timeout()),this,SLOT(onTimer()));
    m_pStereoCalib->m_boardsize.width=(ui->lineEdit_px->text()).toInt();
    m_pStereoCalib->m_boardsize.height=(ui->lineEdit_py->text()).toInt();
    m_pStereoCalib->m_squarelength=(ui->lineEdit_length->text()).toFloat();
    m_pStereoCalib->m_images=(ui->lineEdit_pic_num->text()).toInt();
    qDebug()<<m_pStereoCalib->m_squarelength;
    qDebug()<<m_pStereoCalib->m_boardsize.height;
    if(ui->radioButton_img->isChecked())
    {

        QStringList filenamelist_left=QFileDialog::getOpenFileNames(
                    this,QString::fromLocal8Bit("left camera"),
                    "",
                    tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));
        QStringList filenamelist_right=QFileDialog::getOpenFileNames(
                    this,QString::fromLocal8Bit("right camera"),
                    "",
                    tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));


        if((!filenamelist_left.empty())&&(!filenamelist_right.empty()))
        {

            if(filenamelist_left.count()<m_pStereoCalib->m_images)
            {
                QMessageBox::about(this,"error","number of images is not enough");
            }
            else
            {
                m_pStereoCalib->m_fleftlist=filenamelist_left;
                m_pStereoCalib->m_frightlist=filenamelist_right;
                for(int i=0;i<m_pStereoCalib->m_images;++i)
                {
                    QString filename1=filenamelist_left[i];
                    m_image_left=m_pStereoCalib->getcorners(filename1,m_pStereoCalib->m_objectPointVect1,m_pStereoCalib->m_imagePointVect1);
                    QString filename2=filenamelist_right[i];
                    m_image_right=m_pStereoCalib->getcorners(filename2,m_pStereoCalib->m_objectPointVect2,m_pStereoCalib->m_imagePointVect2);

                }
            }
            m_timer.start(100);
            double err=m_pStereoCalib->CalibStart();
            qDebug()<<err;
            QMessageBox::about(this,"error of Calibration is",QString::number(err));
        }
        else
        {
            QMessageBox::about(this,"error","images are not loaded");
        }


    }
    if(ui->radioButton_cam->isChecked())
    {
        switch (num_camera) {
        case 2:
            m_cap_right>>m_cap_right;

        case 1:
            m_cap_left>>m_src_left;

            break;
        default:
            QMessageBox::about(this,"error","no camera !");
            break;
        }


    }

}

void Dialog::on_radioButton_img_clicked()
{
    ui->radioButton_img->setChecked(true);
}

void Dialog::on_radioButton_cam_clicked()
{
    ui->radioButton_cam->setChecked(true);
}


void Dialog::onTimer_disp(){
    m_cap_left>>m_src_left;
    m_cap_right>>m_src_right;
    m_show_dispa=Mat::zeros(m_src_right.size(),CV_8UC3);

    int flag;
    if(ui->radioButton_BM->isChecked()) flag=0;
    if(ui->radioButton_SGBM->isChecked())flag=1;
    if(ui->radioButton_VAR->isChecked()) flag=2;
    if(ui->radioButton_GC->isChecked()) {flag=3;m_pTimer_disp->stop();};

    m_pStereoMatch->CalcDispa(m_src_left, m_src_right, m_show_dispa, flag);

    if(ui->checkBox_face->isChecked()) findfaces();//人脸识别

    m_image_left=Mat2QImage(m_src_left);
    m_image_right=Mat2QImage(m_src_right);

    QImage imagescaled=m_image_left.scaled(320,240,Qt::KeepAspectRatio);
    ui->label_left->setPixmap(QPixmap::fromImage(imagescaled));
    imagescaled=m_image_right.scaled(320,240,Qt::KeepAspectRatio);
    ui->label_right->setPixmap(QPixmap::fromImage(imagescaled));


    QImage m_image_disp=Mat2QImage(m_show_dispa);
    imagescaled=m_image_disp.scaled(320,240,Qt::KeepAspectRatio);
    ui->label_dispa->setPixmap(QPixmap::fromImage(imagescaled));
}


void Dialog::on_pushButton_DispStart_clicked()
{
    m_timer.stop();

    m_pTimer_disp=new QTimer;
    connect(m_pTimer_disp,SIGNAL(timeout()),this,SLOT(onTimer_disp()));
    m_pStereoMatch->m_param.GetParam();
    {//BM算法赋值
        m_pStereoMatch->BM.state->roi1=m_pStereoMatch->m_param.m_roi1;
        m_pStereoMatch->BM.state->roi2=m_pStereoMatch->m_param.m_roi2;
        m_pStereoMatch->BM.state->preFilterType=CV_STEREO_BM_NORMALIZED_RESPONSE;
        m_pStereoMatch->BM.state->SADWindowSize=(ui->spinBox_SADWS->text()).toInt();
        m_pStereoMatch->BM.state->numberOfDisparities = (ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->BM.state->uniquenessRatio = (ui->spinBox_uniqRatio->text()).toInt();
        m_pStereoMatch->BM.state->disp12MaxDiff=(ui->spinBox_disp12MaxDiff->text()).toInt();
        m_pStereoMatch->BM.state->minDisparity=(ui->spinBox_minDisp->text()).toInt();
        m_pStereoMatch->BM.state->preFilterCap=(ui->spinBox__prefitCap->text()).toInt();
        m_pStereoMatch->BM.state->speckleRange=(ui->spinBox_SpeckRange->text()).toInt();
        m_pStereoMatch->BM.state->textureThreshold=(ui->spinBox_textThres->text()).toInt();
        m_pStereoMatch->BM.state->speckleWindowSize=(ui->spinBox_SpeckWS->text()).toInt();
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();

    }

    {//SGBM算法赋值
        m_pStereoMatch->SGBM.disp12MaxDiff=(ui->spinBox_disp12MaxDiff->text()).toInt();
        m_pStereoMatch->SGBM.minDisparity=(ui->spinBox_minDisp->text()).toInt();
        m_pStereoMatch->SGBM.numberOfDisparities=(ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->SGBM.preFilterCap=(ui->spinBox__prefitCap->text()).toInt();
        m_pStereoMatch->SGBM.SADWindowSize=(ui->spinBox_SADWS->text()).toInt();
        m_pStereoMatch->SGBM.speckleRange=(ui->spinBox_SpeckRange->text()).toInt();
        m_pStereoMatch->SGBM.speckleWindowSize=(ui->spinBox_SpeckWS->text()).toInt();
        m_pStereoMatch->SGBM.uniquenessRatio=(ui->spinBox_uniqRatio->text()).toInt();
        m_pStereoMatch->SGBM.fullDP=false;
        //int cn = m_src_left.channels();
        int cn=3;
        m_pStereoMatch->SGBM.P1=8*cn*m_pStereoMatch->SGBM.SADWindowSize*m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->SGBM.P2=32*cn*m_pStereoMatch->SGBM.SADWindowSize*m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
    }

    {//VAR算法赋值
        m_pStereoMatch->var.minDisp = -1*(ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->var.maxDisp = 0;
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
        /*
        m_pStereoMatch->var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
        m_pStereoMatch->var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
        m_pStereoMatch->var.nIt = 25;

        m_pStereoMatch->var.poly_n = 5;
        m_pStereoMatch->var.poly_sigma = 1.1;
        m_pStereoMatch->var.fi = 15.0f;
        m_pStereoMatch->var.lambda = 0.03f;
        m_pStereoMatch->var.penalization = m_pStereoMatch->var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
        m_pStereoMatch->var.cycle = m_pStereoMatch->var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
        m_pStereoMatch->var.flags = m_pStereoMatch->var.USE_SMART_ID | m_pStereoMatch->var.USE_AUTO_PARAMS | m_pStereoMatch->var.USE_INITIAL_DISPARITY | m_pStereoMatch->var.USE_MEDIAN_FILTERING ;
*/
    }
    {
        m_pStereoMatch->GCState->numberOfDisparities=(ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->GCState->minDisparity=(ui->spinBox_minDisp->text()).toInt();
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
    }
    m_pTimer_disp->start(200);
    ui->pushButton_DispStart->setEnabled(false);
    ui->pushButton_DispStop->setEnabled(true);
    ui->pushButton_CloseCam->setEnabled(false);
    ui->pushButton_Calib->setEnabled(false);
}

void Dialog::on_pushButton_DispStop_clicked()
{
    delete m_pTimer_disp;
    ui->pushButton_DispStart->setEnabled(true);
    ui->pushButton_DispStop->setEnabled(false);
    ui->pushButton_CloseCam->setEnabled(true);
    ui->pushButton_Calib->setEnabled(true);
}


void Dialog::on_radioButton_BM_clicked()
{
    //BM算法赋值
        m_pStereoMatch->BM.state->roi1=m_pStereoMatch->m_param.m_roi1;
        m_pStereoMatch->BM.state->roi2=m_pStereoMatch->m_param.m_roi2;
        m_pStereoMatch->BM.state->preFilterType=CV_STEREO_BM_NORMALIZED_RESPONSE;
        m_pStereoMatch->BM.state->SADWindowSize=(ui->spinBox_SADWS->text()).toInt();
        m_pStereoMatch->BM.state->numberOfDisparities = (ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->BM.state->uniquenessRatio = (ui->spinBox_uniqRatio->text()).toInt();
        m_pStereoMatch->BM.state->disp12MaxDiff=(ui->spinBox_disp12MaxDiff->text()).toInt();
        m_pStereoMatch->BM.state->minDisparity=(ui->spinBox_minDisp->text()).toInt();
        m_pStereoMatch->BM.state->preFilterCap=(ui->spinBox__prefitCap->text()).toInt();
        m_pStereoMatch->BM.state->speckleRange=(ui->spinBox_SpeckRange->text()).toInt();
        m_pStereoMatch->BM.state->textureThreshold=(ui->spinBox_textThres->text()).toInt();
        m_pStereoMatch->BM.state->speckleWindowSize=(ui->spinBox_SpeckWS->text()).toInt();
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
}

void Dialog::on_radioButton_SGBM_clicked()
{
    //SGBM算法赋值
        m_pStereoMatch->SGBM.disp12MaxDiff=(ui->spinBox_disp12MaxDiff->text()).toInt();
        m_pStereoMatch->SGBM.minDisparity=(ui->spinBox_minDisp->text()).toInt();
        m_pStereoMatch->SGBM.numberOfDisparities=(ui->spinBox_numDisp->text()).toInt();
        m_pStereoMatch->SGBM.preFilterCap=(ui->spinBox__prefitCap->text()).toInt();
        m_pStereoMatch->SGBM.SADWindowSize=(ui->spinBox_SADWS->text()).toInt();
        m_pStereoMatch->SGBM.speckleRange=(ui->spinBox_SpeckRange->text()).toInt();
        m_pStereoMatch->SGBM.speckleWindowSize=(ui->spinBox_SpeckWS->text()).toInt();
        m_pStereoMatch->SGBM.uniquenessRatio=(ui->spinBox_uniqRatio->text()).toInt();
        m_pStereoMatch->SGBM.fullDP=false;
        //int cn = m_src_left.channels();
        int cn=1;
        m_pStereoMatch->SGBM.P1=8*cn*m_pStereoMatch->SGBM.SADWindowSize*m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->SGBM.P2=32*cn*m_pStereoMatch->SGBM.SADWindowSize*m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
}

void Dialog::on_radioButton_VAR_clicked()
{
    m_pStereoMatch->var.minDisp = -1*(ui->spinBox_numDisp->text()).toInt();
    m_pStereoMatch->var.maxDisp = 0;
    m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
    /*
    m_pStereoMatch->var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
    m_pStereoMatch->var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    m_pStereoMatch->var.nIt = 25;

    m_pStereoMatch->var.poly_n = 3;
    m_pStereoMatch->var.poly_sigma = 0.0;
    m_pStereoMatch->var.fi = 15.0f;
    m_pStereoMatch->var.lambda = 0.03f;
    m_pStereoMatch->var.penalization = m_pStereoMatch->var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    m_pStereoMatch->var.cycle = m_pStereoMatch->var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    m_pStereoMatch->var.flags = m_pStereoMatch->var.USE_SMART_ID | m_pStereoMatch->var.USE_AUTO_PARAMS | m_pStereoMatch->var.USE_INITIAL_DISPARITY | m_pStereoMatch->var.USE_MEDIAN_FILTERING ;
*/
}

void Dialog::findfaces()
{
    Mat gray;
    vector<Rect> faces;
    CascadeClassifier cascade;
    cascade.load("F:/work/usbcamera/haarcascade_frontalface_alt.xml");
    cvtColor(m_src_left,gray,CV_BGR2GRAY);
    //equalizeHist(gray,gray);
    cascade.detectMultiScale(gray,faces);
    for(unsigned i=0;i<faces.size();i++){
        //**************预测***************
        //double t=(double)getTickCount();
        //Mat faceROI,pca_faceROI,predict_label;
        //resize(gray(faces[i]),faceROI,Size(92,112),0,0,CV_INTER_LINEAR);
        //cout<<(getTickCount()-t)/getTickFrequency()*1000<<endl;
        //faceROI.reshape(1,1).convertTo(faceROI,faceROI.type());
        //pca_face.project(faceROI,pca_faceROI);
        //svm_face.predict(pca_faceROI,predict_label);
        //rectangle(m_src_left,faces[i],Scalar(255,0,0));
        rectangle(m_show_dispa,faces[i],Scalar(255));
        //stringstream ss;
        //ss<<predict_label;
        //putText(gray,ss.str(),Point(faces[i].x,faces[i].y),CV_FONT_NORMAL,gray.cols/200,Scalar(255));
    }

}

void Dialog::on_radioButton_GC_clicked()
{
    m_pStereoMatch->GCState->numberOfDisparities=(ui->spinBox_numDisp->text()).toInt();
    m_pStereoMatch->GCState->minDisparity=(ui->spinBox_minDisp->text()).toInt();
    m_pStereoMatch->m_color_value=(ui->spinBox_ColorValue->text()).toInt();
}
