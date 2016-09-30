#ifndef QIMAGEMAT
#define QIMAGEMAT
#include "qimage.h"
#include "opencv.hpp"
#include "qdebug.h"

using namespace cv;

QImage Mat2QImage(const Mat &);
Mat QImage2Mat(QImage image);



#endif // QIMAGEMAT

