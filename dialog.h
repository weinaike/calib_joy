#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QMessageBox>
#include "opencv.hpp"
#include "qtimer.h"
#include "cstereocalib.h"
#include "cstereomatch.h"
#include "qdebug.h"
#include "cstitch.h"
using namespace cv;
namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    void findfaces();
    void show_image(const Mat & m_src,QImage & imagescaled);
    void show_result(const Mat & m_result);
    void updateflag();

private slots:

    void on_pushButton_CloseCam_clicked();
    void on_pushButton_OpenCam_clicked();
    void on_pushButton_Calib_clicked();
    void on_radioButton_img_clicked();
    void on_radioButton_cam_clicked();

    void onTimer();
    void onTimer_cam();
    void onTimer_disp();

    void on_pushButton_DispStart_clicked();
    void on_pushButton_DispStop_clicked();
    void on_radioButton_BM_clicked();
    void on_radioButton_SGBM_clicked();
    void on_radioButton_VAR_clicked();
    void on_radioButton_GC_clicked();

    void on_lineEdit_py_editingFinished();
    void on_lineEdit_length_editingFinished();
    void on_lineEdit_pic_num_editingFinished();
    void on_lineEdit_px_editingFinished();
    void on_pushButton_chessboard_clicked();

    void on_checkBox_rectify_clicked();
    void on_pushButton_initCamera_clicked();

    void on_checkBox_stitch_clicked();

    void on_pushButton_load_clicked();

    void on_lineEdit_height_cmos_editingFinished();

    void on_lineEdit_width_cmos_editingFinished();

    void on_lineEdit_remapx_editingFinished();

    void on_lineEdit_remapy_editingFinished();

private:
    Ui::Dialog *ui;
    CStereoCalib *m_pStereoCalib;//相机标定
    CStereoMatch *m_pStereoMatch;
    cstitch m_stitchprocess;
    QTimer m_timer;
    QTimer *m_pTimer_disp;

    VideoCapture m_cap_left;//左摄像头
    VideoCapture m_cap_right;//右摄像头
    Mat m_src_left,m_src_right;//摄像头读取的图片
    Mat m_pano; //合并全景图
    Mat m_show_dispa;
    QImage m_image_left,m_image_right;//界面显示的图片
    int num_camera;
    int num_getchessbord;
    int height;
    int width;
    float cmos_hegiht;
    float cmos_width;
    float m_remapx;
    float m_remapy;
};

#endif // DIALOG_H
