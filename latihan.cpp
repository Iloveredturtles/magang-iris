#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct TrackedObject {
    std::string name;
    int hMin, hMax;       
    int lMin, lMax;
    int sMin, sMax;
    int minArea;
    double wRealMm;
    bool wrapHue;          // true only for red (hue wraps around 0/179)
    cv::Scalar color;
};

const std::string PANEL = "control_HSL";

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    std::vector<TrackedObject> objects = {
        //  name              hMin hMax  lMin lMax  sMin sMax  minArea  W_real  wrap   color (BGR)
        { "Red Bottle",        12, 168,   25, 255,   70, 255,    1000,    80.0, true,  cv::Scalar(0,   0, 255) },
        { "Yellow Helmet",     28,  48,   80, 255,   60, 255,    3000,   190.0, false, cv::Scalar(0, 255, 255) },
        { "Pouch Teal",        85, 105,   35, 230,   80, 255,    1500,    60.0, false, cv::Scalar(255,200,  0) },
    };

    int active = 0;
    int fxValue = 614;
    int hMin = 0, hMax = 0, lMin = 0, lMax = 0, sMin = 0, sMax = 0, areaBy100 = 0;

    cv::namedWindow(PANEL, cv::WINDOW_NORMAL);
    cv::resizeWindow(PANEL, 520, 420);
    cv::createTrackbar("Object (0/1/2)", PANEL, &active, (int)objects.size() - 1);
    cv::createTrackbar("H Min",     PANEL, &hMin, 179);
    cv::createTrackbar("H Max",     PANEL, &hMax, 179);
    cv::createTrackbar("L Min",     PANEL, &lMin, 255);
    cv::createTrackbar("L Max",     PANEL, &lMax, 255);
    cv::createTrackbar("S Min",     PANEL, &sMin, 255);
    cv::createTrackbar("S Max",     PANEL, &sMax, 255);
    cv::createTrackbar("Area /100", PANEL, &areaBy100, 200);
    cv::createTrackbar("FX",        PANEL, &fxValue, 2000);

    int lastActive = -1;  

    cv::Mat frame, hls, mask, maskLow, maskHigh;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Mat> maskViews;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        if (active != lastActive) {
            lastActive = active;
            const TrackedObject& o = objects[active];
            cv::setTrackbarPos("H Min",     PANEL, o.hMin);
            cv::setTrackbarPos("H Max",     PANEL, o.hMax);
            cv::setTrackbarPos("L Min",     PANEL, o.lMin);
            cv::setTrackbarPos("L Max",     PANEL, o.lMax);
            cv::setTrackbarPos("S Min",     PANEL, o.sMin);
            cv::setTrackbarPos("S Max",     PANEL, o.sMax);
            cv::setTrackbarPos("Area /100", PANEL, o.minArea / 100);
        } else {
            TrackedObject& o = objects[active];
            o.hMin = hMin;  o.hMax = hMax;
            o.lMin = lMin;  o.lMax = lMax;
            o.sMin = sMin;  o.sMax = sMax;
            o.minArea = areaBy100 * 100;
        }

        cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);

        const double cx = frame.cols / 2.0;
        const double cy = frame.rows / 2.0;
        const double fx = (fxValue > 0) ? (double)fxValue : 1.0;

        maskViews.clear();

        for (size_t k = 0; k < objects.size(); k++) {
            const TrackedObject& obj = objects[k];

            if (obj.wrapHue) {
                // because my red bottle has both ends of the hue wheel, so one range cannot express it
                cv::inRange(hls, cv::Scalar(0, obj.lMin, obj.sMin),
                                 cv::Scalar(obj.hMin, obj.lMax, obj.sMax), maskLow);
                cv::inRange(hls, cv::Scalar(obj.hMax, obj.lMin, obj.sMin),
                                 cv::Scalar(179, obj.lMax, obj.sMax), maskHigh);
                cv::bitwise_or(maskLow, maskHigh, mask);
            } else {
                cv::inRange(hls, cv::Scalar(obj.hMin, obj.lMin, obj.sMin),
                                 cv::Scalar(obj.hMax, obj.lMax, obj.sMax), mask);
            }

            cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
            cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);

            //EACH OBJECT MASK
            cv::Mat view;
            cv::cvtColor(mask, view, cv::COLOR_GRAY2BGR);
            cv::resize(view, view, cv::Size(320, 240));
            cv::putText(view, obj.name, cv::Point(8, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 1);
            if ((int)k == active) {
                cv::rectangle(view, cv::Rect(0, 0, 319, 239), cv::Scalar(0, 255, 0), 2);
            }
            maskViews.push_back(view);
            // ---------------------------------------

            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double bestArea = 0;
            int bestIndex = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area < obj.minArea) continue;
                if (area > bestArea) { bestArea = area; bestIndex = (int)i; }
            }

            if (bestIndex < 0) continue;

            cv::Rect box = cv::boundingRect(contours[bestIndex]);
            cv::rectangle(frame, box, obj.color, 2);

            cv::Moments m = cv::moments(contours[bestIndex]);
            cv::Point2f centroid(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
            if (m.m00 > 0) {
                centroid = cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
            }
            cv::circle(frame, centroid, 4, obj.color, cv::FILLED);

            const double z = (fx * obj.wRealMm) / box.width;
            const double x = ((centroid.x - cx) * z) / fx;
            const double y = ((centroid.y - cy) * z) / fx;

            cv::putText(frame, obj.name, cv::Point(box.x, box.y - 36),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2);
            cv::putText(frame, cv::format("Z = %.1f cm", z / 10.0),
                        cv::Point(box.x, box.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2);
            cv::putText(frame, cv::format("(%.0f, %.0f, %.0f) cm", x/10.0, y/10.0, z/10.0),
                        cv::Point(box.x, box.y - 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, obj.color, 2);
            cv::putText(frame, cv::format("w = %d px  |  W_real = %.0f mm", box.width, obj.wRealMm),
                        cv::Point(box.x, box.y + box.height + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, obj.color, 2);
        }

        cv::putText(frame, "Press [Q] to quit", cv::Point(10, frame.rows - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Normal", frame);

        if (!maskViews.empty()) {
            cv::Mat strip;
            cv::hconcat(maskViews, strip);
            cv::imshow("Object Masks", strip);
        }

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) break;
    }
    return 0;
}