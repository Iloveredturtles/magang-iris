// #include <opencv2/opencv.hpp>

// // ^ webcam

// int main() {
//     cv::VideoCapture cap(0);
//     if (!cap.isOpened()) return -1;

//     cv::Mat frame, hls;
//     std::vector<cv::Mat> channels;

//     while (true) {
//         cap >> frame;

//         cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
//         cv::split(hls, channels);

//         cv::imshow("Normal", frame);
//         cv::imshow("H - which colour", channels[0]);
//         cv::imshow("L - how bright", channels[1]);

//         if (cv::waitKey(1) == 27) break;
//     }
//     return 0;
// }

#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    int hMin = 100, hMax = 130;
    int lMin = 40,  lMax = 220;
    int sMin = 80,  sMax = 255;

    cv::namedWindow("Kontrol", cv::WINDOW_NORMAL);
    cv::createTrackbar("H Min", "Kontrol", &hMin, 179);
    cv::createTrackbar("H Max", "Kontrol", &hMax, 179);
    cv::createTrackbar("L Min", "Kontrol", &lMin, 255);
    cv::createTrackbar("L Max", "Kontrol", &lMax, 255);
    cv::createTrackbar("S Min", "Kontrol", &sMin, 255);
    cv::createTrackbar("S Max", "Kontrol", &sMax, 255);

    cv::Mat frame, hls, mask;

    while (true) {
        cap >> frame;
        cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);

        cv::inRange(hls,
                    cv::Scalar(hMin, lMin, sMin),
                    cv::Scalar(hMax, lMax, sMax),
                    mask);

        cv::imshow("Normal", frame);
        cv::imshow("Mask", mask);

        if (cv::waitKey(1) == 27) break;
    }
    return 0;
}