#include "dialog.h"
#include "ui_dialog.h"
#include "qfiledialog.h"
#include "opencv.hpp"
#include "qdebug.h"
#include "time.h"
#include <QDateTime>
using namespace cv;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{

    ui->setupUi(this);
    num_camera = 0;
    height = (ui->lineEdit_height->text()).toInt();
    width = (ui->lineEdit_width->text()).toInt();

    if(m_cap_left.open(0))
    {
        num_camera++;
        m_cap_left.set(CV_CAP_PROP_FRAME_HEIGHT,height);
        m_cap_left.set(CV_CAP_PROP_FRAME_WIDTH,width);
        m_cap_left>>m_src_left;
        m_cap_left>>m_src_left;
    }
    if(m_cap_right.open(1))
    {
        num_camera++;
        m_cap_right.set(CV_CAP_PROP_FRAME_HEIGHT,height);
        m_cap_right.set(CV_CAP_PROP_FRAME_WIDTH,width);
        m_cap_right>>m_src_right;
        m_cap_right>>m_src_right;
    }

    qDebug()<<"cameras: "<<num_camera<<endl;
    m_pStereoCalib=new CStereoCalib;
    m_pStereoMatch=new CStereoMatch;
    m_pStereoCalib->m_boardsize.width=(ui->lineEdit_px->text()).toInt();
    m_pStereoCalib->m_boardsize.height=(ui->lineEdit_py->text()).toInt();
    m_pStereoCalib->m_squarelength=(ui->lineEdit_length->text()).toFloat();
    m_pStereoCalib->num_used=(ui->lineEdit_pic_num->text()).toInt();

    ui->pushButton_CloseCam->setDisabled(true);
    ui->pushButton_DispStart->setDisabled(true);
    ui->pushButton_DispStop->setDisabled(true);
    ui->pushButton_Calib->setDisabled(false);
    ui->pushButton_OpenCam->setEnabled(true);
    ui->pushButton_chessboard->setEnabled(false);
    num_getchessbord = 0;
}

Dialog::~Dialog()
{
    delete ui;
    delete m_pStereoCalib;
    delete m_pStereoMatch;
    switch (num_camera) {
    case 2:
        m_cap_right.release();
    case 1:
        m_cap_left.release();
    default:
        break;
    }
}



void Dialog::onTimer(){
    switch (num_camera) {
    case 2:
    {
        QImage imagescaled=m_image_right.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_right->setPixmap(QPixmap::fromImage(imagescaled));
    }
    case 1:
    {
        QImage imagescaled=m_image_left.scaled(320,240,Qt::KeepAspectRatio);
        ui->label_left->setPixmap(QPixmap::fromImage(imagescaled));
        break;
    }
    default:
        break;
    }
}

void Dialog::updateflag(){
    m_pStereoCalib->m_boardsize.width=(ui->lineEdit_px->text()).toInt();
    m_pStereoCalib->m_boardsize.height=(ui->lineEdit_py->text()).toInt();
    m_pStereoCalib->m_squarelength=(ui->lineEdit_length->text()).toFloat();
    m_pStereoCalib->num_used=(ui->lineEdit_pic_num->text()).toInt();
    m_pStereoCalib->flag = 0;
    if (ui->checkBox_fisheye->isChecked()){
        if(ui->checkBox_FUIG->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
        if(ui->checkBox_FRE->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if(ui->checkBox_FCC->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_CHECK_COND;
        if(ui->checkBox_FFS->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_FIX_SKEW;
        if(ui->checkBox_FFK1->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_FIX_K1;
        if(ui->checkBox_FFK2->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_FIX_K2;
        if(ui->checkBox_FFK3->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_FIX_K3;
        if(ui->checkBox_FFK4->isChecked())
            m_pStereoCalib->flag |= cv::fisheye::CALIB_FIX_K4;
    }
    else{
        if(ui->checkBox_UIG->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_USE_INTRINSIC_GUESS;
        if(ui->checkBox_FAR->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_ASPECT_RATIO;
        if(ui->checkBox_FPP->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(ui->checkBox_ZTD->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(ui->checkBox_FFL->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_FOCAL_LENGTH;
        if(ui->checkBox_FK1->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K1;
        if(ui->checkBox_FK2->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K2;
        if(ui->checkBox_FK3->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K3;
        if(ui->checkBox_FK4->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K4;
        if(ui->checkBox_FK5->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K5;
        if(ui->checkBox_FK6->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_FIX_K6;
        if(ui->checkBox_RM->isChecked())
            m_pStereoCalib->flag |= CV_CALIB_RATIONAL_MODEL;

    }
}

void Dialog::show_image(const Mat & m_src,QImage & imagescaled){
    QImage m_image;
    Mat temp;
    m_src.copyTo(temp);
    if(ui->radioButton_cam->isChecked()){
        bool found = false;
        vector<Point2f> pointBuf;
        Size sz = Size(m_pStereoCalib->m_boardsize);
        found = findChessboardCorners(temp,sz,pointBuf,
                                      CALIB_CB_FAST_CHECK|CV_CALIB_CB_ADAPTIVE_THRESH);
        if(found){
            /*
            Mat viewGray;
            cvtColor(temp, viewGray, COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                          Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                          */
            drawChessboardCorners(temp,m_pStereoCalib->m_boardsize,Mat(pointBuf),found);
        }
    }
    m_image=Mat2QImage(temp);
    imagescaled=m_image.scaled(320,240,Qt::KeepAspectRatio);
}

void Dialog::onTimer_cam()
{
    //clock_t start,stop;
    //start = clock();
    switch (num_camera) {
    case 2:
    {
        m_cap_right>>m_src_right;
        if(ui->checkBox_rectify->isChecked()){
            Mat temp;
            if(ui->checkBox_remap->isChecked()){
                Mat map1, map2;
                initUndistortRectifyMap(m_pStereoCalib->m_param.m_CameraMat2,
                                        m_pStereoCalib->m_param.m_DistMat2,Mat(),
                                        getOptimalNewCameraMatrix(m_pStereoCalib->m_param.m_CameraMat2,
                                                                  m_pStereoCalib->m_param.m_DistMat2,
                                                                  Size(height,width), 1,
                                                                  Size(height,width), 0 ,false),
                                        Size(height*2,width), CV_16SC2, map1, map2);
                remap(m_src_right, temp, map1, map2, INTER_LINEAR);
            }
            else{
                undistort(m_src_right, temp, m_pStereoCalib->m_param.m_CameraMat2,
                          m_pStereoCalib->m_param.m_DistMat2);
            }
            show_image(temp,m_image_right);
        }else{
            show_image(m_src_right,m_image_right);
        }
        ui->label_right->setPixmap(QPixmap::fromImage(m_image_right));
    }
    case 1:
    {
        m_cap_left>>m_src_left;
        if(ui->checkBox_rectify->isChecked()){
            Mat temp;
            if(ui->checkBox_remap->isChecked()){
                Mat map1, map2;
                initUndistortRectifyMap(m_pStereoCalib->m_param.m_CameraMat1,
                                        m_pStereoCalib->m_param.m_DistMat1,Mat(),
                                        getOptimalNewCameraMatrix(m_pStereoCalib->m_param.m_CameraMat1,
                                                                  m_pStereoCalib->m_param.m_DistMat1,
                                                                  Size(height,width), 1,
                                                                  Size(height,width), 0 ,false),
                                        Size(height*2,width), CV_16SC2, map1, map2);
                remap(m_src_left, temp, map1, map2, INTER_LINEAR);
            }
            else{
                undistort(m_src_left, temp, m_pStereoCalib->m_param.m_CameraMat1,
                          m_pStereoCalib->m_param.m_DistMat1);
            }

            show_image(temp,m_image_left);
        }else{
            show_image(m_src_left,m_image_left);
        }
        ui->label_left->setPixmap(QPixmap::fromImage(m_image_left));
        break;
    }
    default:
        break;
    }
    //stop = clock();
    //double totaltime=(double)(stop-start)/CLOCKS_PER_SEC;
    //qDebug()<<"one loop is "<<totaltime*1000<<"ms"<<endl;

}


void Dialog::on_pushButton_CloseCam_clicked()
{
    m_timer.stop();
    ui->pushButton_CloseCam->setDisabled(true);
    ui->pushButton_DispStart->setDisabled(true);
    ui->pushButton_DispStop->setDisabled(true);
    ui->pushButton_OpenCam->setEnabled(true);
    if(ui->radioButton_cam->isChecked())
        ui->pushButton_chessboard->setEnabled(false);
}

void Dialog::on_pushButton_OpenCam_clicked()
{
    connect(&m_timer,SIGNAL(timeout()),this,SLOT(onTimer_cam()));
    m_timer.start(1);
    ui->pushButton_OpenCam->setDisabled(true);
    ui->pushButton_CloseCam->setEnabled(true);
    ui->pushButton_DispStart->setEnabled(true);
    if(ui->radioButton_cam->isChecked())
        ui->pushButton_chessboard->setEnabled(true);

}

void Dialog::on_pushButton_Calib_clicked()
{
    ui->checkBox_rectify->setChecked(false);
    ui->checkBox_rectify->setEnabled(false);
    ui->pushButton_Calib->setEnabled(false);
    //导入界面参数
    updateflag();
    if(ui->radioButton_img->isChecked())
    {
        connect(&m_timer,SIGNAL(timeout()),this,SLOT(onTimer()));
        switch (num_camera) {
        case 2:
        {
            QStringList filenamelist_right=QFileDialog::getOpenFileNames(
                        this,QString::fromLocal8Bit("right camera"),"",
                        tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));
            if(!filenamelist_right.empty()){
                if(filenamelist_right.count()<m_pStereoCalib->num_used)
                {
                    QMessageBox::about(this,"error","number of images is not enough");
                }
                else
                {
                    m_pStereoCalib->m_frightlist=filenamelist_right;
                    for(int i=0;i<m_pStereoCalib->num_used;++i)
                    {
                        QString filename2=filenamelist_right[i];
                        m_image_right=m_pStereoCalib->getcorners(filename2,
                                m_pStereoCalib->m_objectPointVect2,
                                m_pStereoCalib->m_imagePointVect2);
                    }
                }
            }
            else{
                QMessageBox::about(this,"error","no images");
            }
        }
        case 1:
        {
            QStringList filenamelist_left=QFileDialog::getOpenFileNames(
                        this,QString::fromLocal8Bit("left camera"),"",
                        tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));
            if(!filenamelist_left.empty()){
                if(filenamelist_left.count()<m_pStereoCalib->num_used)
                {
                    QMessageBox::about(this,"error","number of images is not enough");
                }
                else
                {
                    m_pStereoCalib->m_frightlist=filenamelist_left;
                    for(int i=0;i<m_pStereoCalib->num_used;++i)
                    {
                        QString filename1=filenamelist_left[i];
                        m_image_left=m_pStereoCalib->getcorners(filename1,
                                 m_pStereoCalib->m_objectPointVect1,
                                 m_pStereoCalib->m_imagePointVect1);
                    }
                }
            }
            else{
                QMessageBox::about(this,"error","no images");
            }
            break;
        }
        default:
            QMessageBox::about(this,"error","no camera");
            break;
        }
        m_timer.start(100);
        double err;
        if(ui->checkBox_fisheye->isChecked())
            err = m_pStereoCalib->FisheyeStart();
        else
            err = m_pStereoCalib->CalibStart();
    }
    if(ui->radioButton_cam->isChecked())
    {
         QMessageBox::about(this,"error","切换到图片模式");
    }
    ui->checkBox_rectify->setEnabled(true);
    ui->pushButton_Calib->setEnabled(true);
    on_checkBox_rectify_clicked();
}

void Dialog::on_radioButton_img_clicked()
{
    ui->radioButton_img->setChecked(true);
    ui->pushButton_chessboard->setEnabled(false);
}

void Dialog::on_radioButton_cam_clicked()
{
    ui->radioButton_cam->setChecked(true);
    if(ui->pushButton_CloseCam->isEnabled())
        ui->pushButton_chessboard->setEnabled(true);
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
        m_pStereoMatch->SGBM.P1=8*cn*m_pStereoMatch->SGBM.SADWindowSize
                *m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->SGBM.P2=32*cn*m_pStereoMatch->SGBM.SADWindowSize
                *m_pStereoMatch->SGBM.SADWindowSize;
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
        m_pStereoMatch->SGBM.P1=8*cn*m_pStereoMatch->SGBM.SADWindowSize
                *m_pStereoMatch->SGBM.SADWindowSize;
        m_pStereoMatch->SGBM.P2=32*cn*m_pStereoMatch->SGBM.SADWindowSize
                *m_pStereoMatch->SGBM.SADWindowSize;
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

void Dialog::on_lineEdit_py_editingFinished()
{
    m_pStereoCalib->m_boardsize.height=(ui->lineEdit_py->text()).toInt();
}

void Dialog::on_lineEdit_px_editingFinished()
{
    m_pStereoCalib->m_boardsize.width=(ui->lineEdit_px->text()).toInt();
}

void Dialog::on_lineEdit_length_editingFinished()
{
    m_pStereoCalib->m_squarelength=(ui->lineEdit_length->text()).toFloat();
}

void Dialog::on_lineEdit_pic_num_editingFinished()
{
    m_pStereoCalib->num_used=(ui->lineEdit_pic_num->text()).toInt();
}

void Dialog::on_pushButton_chessboard_clicked()
{
    QDateTime time(QDateTime::currentDateTime());//获取系统现在的时间
    QString str = time.toString("yyyy_MM_dd_hh_mm_ss"); //设置显示格式
    std::string filename = str.toStdString();
    switch (num_camera) {
    case 2:
        imwrite("right_"+filename+".jpg",m_src_right);
    case 1:
        imwrite("left_"+filename+".jpg",m_src_left);
        break;
    default:
        break;
    }
    num_getchessbord++;

    ui->label_info->setText("第"+QString::number(num_getchessbord,10)+"张");
}

void Dialog::on_checkBox_rectify_clicked()
{
    m_pStereoCalib->m_param.GetParam();

    if (!m_pStereoCalib->m_param.m_CameraMat1.empty())
    {
        double fovx,fovy,focallength;
        Point2d principalPoint;
        double aspectRatio;
        calibrationMatrixValues(m_pStereoCalib->m_param.m_CameraMat1,
                            Size(height,width), 4.8, 3.6,
                            fovx, fovy, focallength,principalPoint,aspectRatio);
        QString str="焦距: "+QString::number(focallength)+";      "
                +"视场角: ["+QString::number(fovx)+", "+QString::number(fovy)+"]\n"
                +"主点: ["+QString::number(principalPoint.x)+",   "
                +QString::number(principalPoint.y)+"];     "
                +"fx/fy:"+QString::number(aspectRatio)+"\n "
                +"校准误差:"+QString::number(m_pStereoCalib->m_err1)+"\n";
        ui->label_left_info->setText(str);
    }

    if (!m_pStereoCalib->m_param.m_CameraMat2.empty())
    {
        double fovx,fovy,focallength;
        Point2d principalPoint;
        double aspectRatio;
        calibrationMatrixValues(m_pStereoCalib->m_param.m_CameraMat2,
                            Size(height,width), 4.8, 3.6,
                            fovx, fovy, focallength,principalPoint,aspectRatio);
        QString str="焦距: "+QString::number(focallength)+";      "
                +"视场角: ["+QString::number(fovx)+", "+QString::number(fovy)+"]\n"
                +"主点: ["+QString::number(principalPoint.x)+",   "+
                QString::number(principalPoint.y)+"];     "
                +"fx/fy:"+QString::number(aspectRatio)+"\n "
                +"校准误差:"+QString::number(m_pStereoCalib->m_err2)+"\n";
        ui->label_right_info->setText(str);
    }
}



void Dialog::on_pushButton_initCamera_clicked()
{
    m_timer.stop();
    ui->pushButton_CloseCam->setEnabled(false);
    ui->pushButton_OpenCam->setEnabled(true);
    m_cap_left.release();
    m_cap_right.release();
    num_camera = 0;
    height = (ui->lineEdit_height->text()).toInt();
    width = (ui->lineEdit_width->text()).toInt();


    if(m_cap_left.open(0))
    {
        num_camera++;
        m_cap_left.set(CV_CAP_PROP_FRAME_HEIGHT,height);
        m_cap_left.set(CV_CAP_PROP_FRAME_WIDTH,width);
        m_cap_left>>m_src_left;
        m_cap_left>>m_src_left;
        m_cap_left>>m_src_left;
    }
    if(m_cap_right.open(1))
    {
        num_camera++;
        m_cap_right.set(CV_CAP_PROP_FRAME_HEIGHT,height);
        m_cap_right.set(CV_CAP_PROP_FRAME_WIDTH,width);
        m_cap_right>>m_src_right;
        m_cap_right>>m_src_right;
    }
}
