#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tracker {


constexpr int HUE_MAX = 179;


// object is detected
struct Detection {
    bool        found    = false;
    cv::Rect    box;
    cv::Point2f centroid{0.f, 0.f};
    double      areaPx   = 0.0;
    cv::Point3d posMm{0.0, 0.0, 0.0};   // X, Y, Z in millimetres
};

//mask cleaning
struct MorphSettings {
    int erodeIter  = 2;
    int dilateIter = 4;
};


class CameraIntrinsics {
public:
    bool loadFromYaml(const std::string& path);

    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }

    const cv::Mat&     cameraMatrix() const { return cameraMatrix_; }
    const cv::Mat&     distCoeffs()   const { return distCoeffs_; }
    const std::string& error()        const { return error_; }

private:
    bool fail(const std::string& why) { error_ = why; return false; }

    cv::Mat     cameraMatrix_;
    cv::Mat     distCoeffs_;
    double      fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    std::string error_;
};

inline bool CameraIntrinsics::loadFromYaml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return fail("cannot open " + path);
    }
    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
    fs.release();

    if (cameraMatrix_.empty() || cameraMatrix_.rows != 3 || cameraMatrix_.cols != 3) {
        return fail("camera_matrix missing or not 3x3 in " + path);
    }
    cameraMatrix_.convertTo(cameraMatrix_, CV_64F);
    if (!distCoeffs_.empty()) {
        distCoeffs_.convertTo(distCoeffs_, CV_64F);
    }

    fx_ = cameraMatrix_.at<double>(0, 0);
    fy_ = cameraMatrix_.at<double>(1, 1);
    cx_ = cameraMatrix_.at<double>(0, 2);
    cy_ = cameraMatrix_.at<double>(1, 2);

    if (fx_ <= 0.0 || fy_ <= 0.0) {
        return fail("focal length is not valid in " + path);
    }
    return true;
}

//tracking each colors

class TrackedObject {
public:
    std::string name;
    int         hMin, hMax;
    int         lMin, lMax;
    int         sMin, sMax;
    int         minArea;
    double      wRealMm;
    cv::Scalar  color;

    TrackedObject(std::string name_, int hMin_, int hMax_, int lMin_, int lMax_,
                  int sMin_, int sMax_, int minArea_, double wRealMm_, cv::Scalar color_)
        : name(std::move(name_)),
          hMin(hMin_), hMax(hMax_), lMin(lMin_), lMax(lMax_),
          sMin(sMin_), sMax(sMax_), minArea(minArea_),
          wRealMm(wRealMm_), color(color_) {}

    virtual ~TrackedObject() = default;

    virtual cv::Mat buildMask(const cv::Mat& hls) const {
        cv::Mat mask;
        cv::inRange(hls, cv::Scalar(hMin, lMin, sMin),
                         cv::Scalar(hMax, lMax, sMax), mask);
        return mask;
    }

    void detect(const cv::Mat& hls, const CameraIntrinsics& cam,
                const MorphSettings& morph);

    const Detection& detection() const { return detection_; }
    const cv::Mat&   mask()      const { return mask_; }

protected:
    Detection detection_;
    cv::Mat   mask_;

private:
    void cleanMask(const MorphSettings& morph);
    int  largestContour(const std::vector<std::vector<cv::Point>>& contours) const;
    static cv::Point2f centroidOf(const std::vector<cv::Point>& contour, const cv::Rect& box);
    cv::Point3d projectToMm(const CameraIntrinsics& cam) const;
};

// Erode removes specks, dilate fills holes, the second erode restores the size.
inline void TrackedObject::cleanMask(const MorphSettings& morph) {
    const cv::Mat kernel;              // empty = the default 3x3
    const cv::Point anchor(-1, -1);    // centred on itself
    cv::erode (mask_, mask_, kernel, anchor, morph.erodeIter);
    cv::dilate(mask_, mask_, kernel, anchor, morph.dilateIter);
    cv::erode (mask_, mask_, kernel, anchor, morph.erodeIter);
}

// Biggest blob that clears the noise threshold, or -1 if there is none.
inline int TrackedObject::largestContour(
        const std::vector<std::vector<cv::Point>>& contours) const {
    double bestArea  = 0.0;
    int    bestIndex = -1;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area < minArea || area <= bestArea) {
            continue;
        }
        bestArea  = area;
        bestIndex = static_cast<int>(i);
    }
    return bestIndex;
}
inline cv::Point2f TrackedObject::centroidOf(const std::vector<cv::Point>& contour,
                                             const cv::Rect& box) {
    const cv::Moments m = cv::moments(contour);
    if (m.m00 <= 0.0) {
        return cv::Point2f(box.x + box.width  * 0.5f,
                           box.y + box.height * 0.5f);
    }
    return cv::Point2f(static_cast<float>(m.m10 / m.m00),
                       static_cast<float>(m.m01 / m.m00));
}


//   Z = fx * W_real / w_pixelb , X = (x_pixel - cx) * Z / fx ,Y = (y_pixel - cy) * Z / fy
inline cv::Point3d TrackedObject::projectToMm(const CameraIntrinsics& cam) const {
    const double z = (cam.fx() * wRealMm) / detection_.box.width;
    const double x = ((detection_.centroid.x - cam.cx()) * z) / cam.fx();
    const double y = ((detection_.centroid.y - cam.cy()) * z) / cam.fy();
    return cv::Point3d(x, y, z);
}

inline void TrackedObject::detect(const cv::Mat& hls, const CameraIntrinsics& cam,
                                  const MorphSettings& morph) {
    mask_ = buildMask(hls);
    cleanMask(morph);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    detection_ = Detection();
    const int index = largestContour(contours);
    if (index < 0) {
        return;                        // not in frame, nothing to draw
    }

    detection_.found    = true;
    detection_.areaPx   = cv::contourArea(contours[index]);
    detection_.box      = cv::boundingRect(contours[index]);
    detection_.centroid = centroidOf(contours[index], detection_.box);
    detection_.posMm    = projectToMm(cam);
}


// SUBCLASS for my red bottle hue sits at two range
// For this class hMin is the top of the low band and hMax the bottom of the high band

class WrappedHueObject : public TrackedObject {
public:
    using TrackedObject::TrackedObject;

    cv::Mat buildMask(const cv::Mat& hls) const override {
        cv::Mat lowBand, highBand, mask;
        cv::inRange(hls, cv::Scalar(0, lMin, sMin),
                         cv::Scalar(hMin, lMax, sMax), lowBand);
        cv::inRange(hls, cv::Scalar(hMax, lMin, sMin),
                         cv::Scalar(HUE_MAX, lMax, sMax), highBand);
        cv::bitwise_or(lowBand, highBand, mask);
        return mask;
    }
};

}  // namespace tracker

#endif  