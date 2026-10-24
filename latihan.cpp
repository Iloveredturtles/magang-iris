#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    int hMin = 100, hMax = 130;
    int lMin = 40,  lMax = 220;
    int sMin = 80,  sMax = 255;
    int minArea = 10;

    cv::namedWindow("control_HSL", cv::WINDOW_NORMAL);
    cv::resizeWindow("control_HSL", 500, 350);
    cv::createTrackbar("H Min", "control_HSL", &hMin, 179);
    cv::createTrackbar("H Max", "control_HSL", &hMax, 179);
    cv::createTrackbar("L Min", "control_HSL", &lMin, 255);
    cv::createTrackbar("L Max", "control_HSL", &lMax, 255);
    cv::createTrackbar("S Min", "control_HSL", &sMin, 255);
    cv::createTrackbar("S Max", "control_HSL", &sMax, 255);
    cv::createTrackbar("Area /100", "control_HSL", &minArea, 100);

    const cv::Scalar CYAN(255, 255, 0);

    cv::Mat frame, hls, mask;
    std::vector<std::vector<cv::Point>> contours;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
        cv::inRange(hls, cv::Scalar(hMin, lMin, sMin),
                         cv::Scalar(hMax, lMax, sMax), mask);

        cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
        cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);

        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double bestArea = 0;
        int bestIndex = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area < minArea * 100) continue;
            if (area > bestArea) { bestArea = area; bestIndex = (int)i; }
        }

        if (bestIndex >= 0) {
            cv::Rect box = cv::boundingRect(contours[bestIndex]);
            cv::rectangle(frame, box, CYAN, 2);

            std::string label = "w = " + std::to_string(box.width) + " px";
            cv::putText(frame, label, cv::Point(box.x, box.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, CYAN, 2);
        }

        cv::putText(frame, "Press [Q] to quit", cv::Point(10, frame.rows - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Normal", frame);
        cv::imshow("Mask", mask);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) break;
    }
    return 0;
}