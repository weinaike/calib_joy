#include "cstitch.h"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
using namespace cv::detail;
cstitch::cstitch()
{
}

Stitcher::Status cstitch::simplestitch(std::vector<Mat> &images, Mat &pano){
    Stitcher stitcher = Stitcher::createDefault(false);
    Stitcher::Status status = stitcher.stitch(images, pano);
    return status;
}


Stitcher::Status cstitch::detailstitch(std::vector<Mat> &images,Mat &pano)
{

    string ba_refine_mask = "xxxxx";
    string warp_type = "spherical";
    float match_conf = 0.1f;
    float blend_strength = 5;
    Ptr<FeaturesFinder> finder;
    finder = new SurfFeaturesFinder();
    int num_images = static_cast<int>(images.size());
    vector<ImageFeatures> features(num_images);
    vector<Size> full_img_sizes(num_images);
    for (int i = 0; i < num_images; ++i)
    {
        full_img_sizes[i] = images[i].size();
        (*finder)(images[i], features[i]);
        features[i].img_idx = i;
        cout<<"Features in image #" << i+1 << ": " << features[i].keypoints.size()<<endl;
    }
    finder->collectGarbage();
    vector<MatchesInfo> pairwise_matches;
    BestOf2NearestMatcher matcher(false, match_conf);
    matcher(features, pairwise_matches);
    matcher.collectGarbage();

    HomographyBasedEstimator estimator;
    vector<CameraParams> cameras;
    estimator(features, pairwise_matches, cameras);

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    cout<<"cameras"<<cameras.size()<<endl;
    Ptr<detail::BundleAdjusterBase> adjuster;
    adjuster = new detail::BundleAdjusterRay();
    adjuster->setConfThresh(0.2);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    (*adjuster)(features, pairwise_matches, cameras);

    vector<double> focals;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        focals.push_back(cameras[i].focal);
    }
    cout<<"left focal :"<<focals[0]<<" / rightfocal :"<<focals[1]<<endl;
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    cout<<"warped_image_scale"<<warped_image_scale<<endl;
    vector<Point> corners(num_images);
    vector<Mat> masks_warped(num_images);
    vector<Mat> images_warped(num_images);
    vector<Size> sizes(num_images);
    vector<Mat> masks(num_images);
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    // Warp images and their masks

    Ptr<WarperCreator> warper_creator;
    warper_creator = new cv::PlaneWarper();
    double seam_work_aspect = 1;
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;

        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();

        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;

    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {

        Size img_size = images[img_idx].size();
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);

        // Warp the current image
        warper->warp(images[img_idx], K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

        // Compensate exposure

        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        mask.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size());
        mask_warped = seam_mask & mask_warped;

        if (blender.empty())
        {
            blender = Blender::createDefault(Blender::MULTI_BAND, false);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, false);
            MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
            mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));

            blender->prepare(corners, sizes);
        }

        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    if(!pano.data)
    {
        cout<<"nothing"<<endl;
    }
    Mat result_mask;
    blender->blend(pano, result_mask);
    cout<<pano.size();
    if(!pano.data)
    {
        cout<<"nothing"<<endl;
    }
    return Stitcher::Status::OK;
}
