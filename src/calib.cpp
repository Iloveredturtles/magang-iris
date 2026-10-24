// [SPACE] take a snapshot when the board is detected
// [C]     run the calibration and save the file
// [R]     throw away all snapshots and start again
// [Q]     quit

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>


const cv::Size BOARD_SIZE(8, 6);


const float SQUARE_SIZE_MM = 10.0f;

const int MIN_SNAPSHOTS = 15;
const std::string OUT_PATH = "yaml/kalibrasi.yaml";

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "cannot open webcam\n";
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // where the corners are in the real world, on a flat board at z = 0
    std::vector<cv::Point3f> boardPoints;
    for (int row = 0; row < BOARD_SIZE.height; row++) {
        for (int col = 0; col < BOARD_SIZE.width; col++) {
            boardPoints.push_back(cv::Point3f(col * SQUARE_SIZE_MM,
                                              row * SQUARE_SIZE_MM, 0.0f));
        }
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;   // real world
    std::vector<std::vector<cv::Point2f>> imagePoints;    // on screen
    cv::Size imageSize;

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corners;

    std::cout << "board " << BOARD_SIZE.width << "x" << BOARD_SIZE.height
              << " inner corners, square " << SQUARE_SIZE_MM << " mm\n"
              << "take at least " << MIN_SNAPSHOTS
              << " shots from different angles and distances\n\n";

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        imageSize = frame.size();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners,
                         cv::CALIB_CB_ADAPTIVE_THRESH |
                         cv::CALIB_CB_NORMALIZE_IMAGE |
                         cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::drawChessboardCorners(frame, BOARD_SIZE, corners, found);
        }

        cv::putText(frame, cv::format("snapshots: %d / %d",
                                      (int)imagePoints.size(), MIN_SNAPSHOTS),
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);
        cv::putText(frame, found ? "BOARD FOUND - press [SPACE]" : "no board",
                    cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
        cv::putText(frame, "[SPACE] shot   [C] calibrate   [R] reset   [Q] quit",
                    cv::Point(10, frame.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(200, 200, 200), 1);

        cv::imshow("Calibration", frame);
        const int key = cv::waitKey(1) & 0xFF;

        if (key == 'q' || key == 'Q' || key == 27) break;

        if (key == ' ' && found) {
          
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                 30, 0.001));
            imagePoints.push_back(corners);
            objectPoints.push_back(boardPoints);
            std::cout << "snapshot " << imagePoints.size() << " saved\n";
        }

        if (key == 'r' || key == 'R') {
            imagePoints.clear();
            objectPoints.clear();
            std::cout << "cleared\n";
        }

        if (key == 'c' || key == 'C') {
            if ((int)imagePoints.size() < MIN_SNAPSHOTS) {
                std::cout << "only " << imagePoints.size()
                          << " snapshots, need " << MIN_SNAPSHOTS << "\n";
                continue;
            }

            cv::Mat cameraMatrix, distCoeffs;
            std::vector<cv::Mat> rvecs, tvecs;

            double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                             cameraMatrix, distCoeffs, rvecs, tvecs);

            cameraMatrix.convertTo(cameraMatrix, CV_64F);
            distCoeffs.convertTo(distCoeffs, CV_64F);

            double fx = cameraMatrix.at<double>(0, 0);
            double fy = cameraMatrix.at<double>(1, 1);
            double cx = cameraMatrix.at<double>(0, 2);
            double cy = cameraMatrix.at<double>(1, 2);

            cv::FileStorage fs(OUT_PATH, cv::FileStorage::WRITE);
            if (!fs.isOpened()) {
                std::cerr << "cannot write " << OUT_PATH
                          << " - does the yaml/ folder exist?\n";
                continue;
            }
            fs << "image_width"              << imageSize.width;
            fs << "image_height"             << imageSize.height;
            fs << "board_width"              << BOARD_SIZE.width;
            fs << "board_height"             << BOARD_SIZE.height;
            fs << "square_size"              << SQUARE_SIZE_MM;
            fs << "snapshot_count"           << (int)imagePoints.size();
            fs << "reprojection_error"       << rms;
            fs << "camera_matrix"            << cameraMatrix;
            fs << "distortion_coefficients"  << distCoeffs;
            fs << "focal_length_x"           << fx;
            fs << "focal_length_y"           << fy;
            fs << "principal_point_x"        << cx;
            fs << "principal_point_y"        << cy;
            fs.release();

            std::cout << "\n=== RESULT ===\n"
                      << "fx = " << fx << "\nfy = " << fy
                      << "\ncx = " << cx << "\ncy = " << cy
                      << "\nreprojection error = " << rms << " px\n"
                      << "saved to " << OUT_PATH << "\n";
            if (rms > 1.0) {
                std::cout << "error above 1.0 px - retake with more variety\n";
            }
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}