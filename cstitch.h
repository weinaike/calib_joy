#ifndef CSTITCH_H
#define CSTITCH_H
#include <opencv.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <vector>
using namespace cv;
using namespace std;
class cstitch
{
public:
    cstitch();
    Stitcher::Status simplestitch(std::vector<Mat> &images,Mat &pano);
    Stitcher::Status detailstitch(std::vector<Mat> &images,Mat &pano);

};

#endif // CSTITCH_H
