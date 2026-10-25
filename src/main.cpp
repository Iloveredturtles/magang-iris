#include "tracker.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

namespace {

using tracker::CameraIntrinsics;
using tracker::Detection;
using tracker::MorphSettings;
using tracker::SolidColorObject;
using tracker::TrackedObject;
using tracker::GradientColorObject;

using ObjectList = std::vector<std::unique_ptr<TrackedObject>>;

const std::string CALIB_PATH  = "yaml/kalibrasi.yaml";
const std::string PANEL       = "control_HSL";
const std::string MAIN_WINDOW = "Normal";
const std::string MASK_WINDOW = "Object Masks";

const std::string DOCS_DIR = "docs";
const std::string LOG_PATH = "docs/log_koordinat.yaml";

constexpr int FRAME_WIDTH  = 640;
constexpr int FRAME_HEIGHT = 480;

constexpr int MASK_TILE_WIDTH  = 240;
constexpr int MASK_TILE_HEIGHT = 180;

constexpr int FONT = cv::FONT_HERSHEY_SIMPLEX;


constexpr int LABEL_NAME_DY   = -36;
constexpr int LABEL_DIST_DY   = -20;
constexpr int LABEL_COORD_DY  = -4;
constexpr int LABEL_WIDTH_DY  = 16;

double toCm(double mm) { return mm / 10.0; }

// one row of the 3D log written on every capture
struct LogEntry {
    int         capture;
    std::string object;
    double      xMm, yMm, zMm;
    int         widthPx;
};

// FileStorage cannot append, so the whole log is rewritten each time
bool writeLog(const std::vector<LogEntry>& entries) {
    cv::FileStorage fs(LOG_PATH, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "cannot write " << LOG_PATH << "\n";
        return false;
    }
    fs << "unit" << "millimetre";
    fs << "entries" << "[";
    for (const LogEntry& e : entries) {
        fs << "{";
        fs << "capture"  << e.capture;
        fs << "object"   << e.object;
        fs << "X_mm"     << e.xMm;
        fs << "Y_mm"     << e.yMm;
        fs << "Z_mm"     << e.zMm;
        fs << "width_px" << e.widthPx;
        fs << "}";
    }
    fs << "]";
    fs.release();
    return true;
}

// start numbering after the highest capture_N.png already in docs/
int lastCaptureNumber() {
    int highest = 0;
    if (!std::filesystem::exists(DOCS_DIR)) {
        return 0;
    }
    for (const auto& entry : std::filesystem::directory_iterator(DOCS_DIR)) {
        const std::string name = entry.path().filename().string();
        int n = 0;
        if (std::sscanf(name.c_str(), "capture_%d.png", &n) == 1 && n > highest) {
            highest = n;
        }
    }
    return highest;
}


ObjectList createObjects() {
    ObjectList objects;

    //width is just a placehorder, replace it with ur own
    //hMin hMax  lMin lMax  sMin sMax  area  W_real  color (BGR)
    objects.push_back(std::make_unique<GradientColorObject>(
        "Red Square",     12, 168,   25, 255,   70, 255,  800,  45.0, cv::Scalar(  0,   0, 255)));

    objects.push_back(std::make_unique<SolidColorObject>(
        "Teal Inhaler",   70,  98,   35, 230,   60, 255,  500,  25.0, cv::Scalar(200, 200,   0)));

    objects.push_back(std::make_unique<SolidColorObject>(
        "Blue Speaker",  102, 135,   35, 220,   70, 255, 1500,  85.0, cv::Scalar(255,  80,   0)));
    return objects;
}


// choose object using the first slider
struct ControlPanel {
    int active     = 0;
    int hMin = 0, hMax = 0;
    int lMin = 0, lMax = 0;
    int sMin = 0, sMax = 0;
    int areaBy100  = 0;
    int erodeIter  = 2;
    int dilateIter = 4;
    int warnZcm    = 10;              // warn when an object is closer than this
    int lastActive = -1;             

    void create(int objectCount) {
        cv::namedWindow(PANEL, cv::WINDOW_NORMAL);
        cv::resizeWindow(PANEL, 520, 460);
        cv::createTrackbar("Object",    PANEL, &active, objectCount - 1);
        cv::createTrackbar("H Min",     PANEL, &hMin, tracker::HUE_MAX);
        cv::createTrackbar("H Max",     PANEL, &hMax, tracker::HUE_MAX);
        cv::createTrackbar("L Min",     PANEL, &lMin, 255);
        cv::createTrackbar("L Max",     PANEL, &lMax, 255);
        cv::createTrackbar("S Min",     PANEL, &sMin, 255);
        cv::createTrackbar("S Max",     PANEL, &sMax, 255);
        cv::createTrackbar("Area /100", PANEL, &areaBy100, 200);
        cv::createTrackbar("Erode",     PANEL, &erodeIter, 10);
        cv::createTrackbar("Dilate",    PANEL, &dilateIter, 10);
        cv::createTrackbar("Warn Z cm", PANEL, &warnZcm, 200);
    }

    
    void sync(ObjectList& objects) {
        TrackedObject& object = *objects[active];
        if (active != lastActive) {
            lastActive = active;
            cv::setTrackbarPos("H Min",     PANEL, object.hMin);
            cv::setTrackbarPos("H Max",     PANEL, object.hMax);
            cv::setTrackbarPos("L Min",     PANEL, object.lMin);
            cv::setTrackbarPos("L Max",     PANEL, object.lMax);
            cv::setTrackbarPos("S Min",     PANEL, object.sMin);
            cv::setTrackbarPos("S Max",     PANEL, object.sMax);
            cv::setTrackbarPos("Area /100", PANEL, object.minArea / 100);
            return;
        }
        object.hMin    = hMin;   object.hMax = hMax;
        object.lMin    = lMin;   object.lMax = lMax;
        object.sMin    = sMin;   object.sMax = sMax;
        object.minArea = areaBy100 * 100;
    }

    MorphSettings morph() const { return MorphSettings{erodeIter, dilateIter}; }
};


void drawDetection(cv::Mat& frame, const TrackedObject& object, bool tooClose) {
    const Detection& d = object.detection();

    // red and thicker when the object is too close
    const cv::Scalar boxColor = tooClose ? cv::Scalar(0, 0, 255) : object.color;

    cv::rectangle(frame, d.box, boxColor, tooClose ? 3 : 2);
    cv::circle(frame, d.centroid, 4, object.color, cv::FILLED);

    const auto label = [&](const std::string& text, int dy, double scale) {
        cv::putText(frame, text, cv::Point(d.box.x, d.box.y + dy),
                    FONT, scale, boxColor, 2);
    };

    label(object.name, LABEL_NAME_DY, 0.50);
    label(cv::format("Z = %.1f cm", toCm(d.posMm.z)), LABEL_DIST_DY, 0.50);
    label(cv::format("(%.0f, %.0f, %.0f) cm",
                     toCm(d.posMm.x), toCm(d.posMm.y), toCm(d.posMm.z)),
          LABEL_COORD_DY, 0.45);
    label(cv::format("w = %d px  |  W_real = %.0f mm", d.box.width, object.wRealMm),
          d.box.height + LABEL_WIDTH_DY, 0.45);

    if (tooClose) {
        label("TOO CLOSE", d.box.height - 8, 0.50);
    }
}

// All object masks,tuning outlined in green.
cv::Mat buildMaskStrip(const ObjectList& objects, int active) {
    std::vector<cv::Mat> tiles;
    tiles.reserve(objects.size());

    for (std::size_t i = 0; i < objects.size(); ++i) {
        const TrackedObject& object = *objects[i];

        cv::Mat tile;
        cv::cvtColor(object.mask(), tile, cv::COLOR_GRAY2BGR);
        cv::resize(tile, tile, cv::Size(MASK_TILE_WIDTH, MASK_TILE_HEIGHT));
        cv::putText(tile, object.name, cv::Point(8, 20), FONT, 0.5, object.color, 1);

        if (static_cast<int>(i) == active) {
            cv::rectangle(tile, cv::Rect(0, 0, MASK_TILE_WIDTH - 1, MASK_TILE_HEIGHT - 1),
                          cv::Scalar(0, 255, 0), 2);
        }
        tiles.push_back(tile);
    }

    cv::Mat strip;
    cv::hconcat(tiles, strip);
    return strip;
}


cv::Mat undistorted(const cv::Mat& frame, const CameraIntrinsics& cam) {
    if (cam.distCoeffs().empty()) {
        return frame.clone();
    }
    cv::Mat result;
    cv::undistort(frame, result, cam.cameraMatrix(), cam.distCoeffs());
    return result;
}

bool quitRequested(int key) {
    return key == 'q' || key == 'Q' || key == 27;
}

bool captureRequested(int key) {
    return key == ' ';
}

}  // namespace

int main() {
    CameraIntrinsics cam;
    if (!cam.loadFromYaml(CALIB_PATH)) {
        std::cerr << "ERROR: " << cam.error() << "\n\n"
                  << "Run the calibration first:\n  ./calib\n";
        return -1;
    }
    std::cout << "Calibration loaded from " << CALIB_PATH << "\n"
              << "  fx = " << cam.fx() << "   fy = " << cam.fy() << "\n"
              << "  cx = " << cam.cx() << "   cy = " << cam.cy() << "\n\n";

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: cannot open the webcam\n";
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    ObjectList objects = createObjects();

    ControlPanel panel;
    panel.create(static_cast<int>(objects.size()));

    std::filesystem::create_directories(DOCS_DIR);

    // fps timer
    int64 lastTick = cv::getTickCount();

    std::vector<LogEntry> logEntries;
    int captureCount = lastCaptureNumber();

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        panel.sync(objects);

        cv::Mat display = undistorted(frame, cam);
        cv::Mat hls;
        cv::cvtColor(display, hls, cv::COLOR_BGR2HLS);

        // track the nearest object that is closer than the warning limit
        std::string nearestName;
        double nearestZcm  = 0.0;
        bool   anyTooClose = false;

        for (auto& object : objects) {
            object->detect(hls, cam, panel.morph());
            const Detection& d = object->detection();
            if (!d.found) {
                continue;
            }

            const double zCm     = toCm(d.posMm.z);
            const bool   tooClose = zCm < panel.warnZcm;

            if (tooClose && (!anyTooClose || zCm < nearestZcm)) {
                anyTooClose = true;
                nearestZcm  = zCm;
                nearestName = object->name;
            }
            drawDetection(display, *object, tooClose);
        }

        // fps from the time since the last frame
        const int64  now = cv::getTickCount();
        const double fps = cv::getTickFrequency() / (now - lastTick);
        lastTick = now;
        cv::putText(display, cv::format("FPS: %.1f", fps), cv::Point(10, 22),
                    FONT, 0.6, cv::Scalar(0, 255, 255), 2);


        cv::putText(display, "[SPACE] capture   [Q] quit",
                    cv::Point(10, display.rows - 46),
                    FONT, 0.5, cv::Scalar(255, 255, 255), 1);

        // red bar at the bottom when something is too close
        if (anyTooClose) {
            cv::rectangle(display, cv::Rect(0, display.rows - 34, display.cols, 34),
                          cv::Scalar(0, 0, 200), cv::FILLED);
            cv::putText(display, cv::format("PROXIMITY WARNING: %s at %.1f cm",
                                            nearestName.c_str(), nearestZcm),
                        cv::Point(12, display.rows - 11),
                        FONT, 0.55, cv::Scalar(255, 255, 255), 2);
        }

        cv::imshow(MAIN_WINDOW, display);
        cv::imshow(MASK_WINDOW, buildMaskStrip(objects, panel.active));

        const int key = cv::waitKey(1) & 0xFF;
        if (quitRequested(key)) {
            break;
        }

        // save the frame as docs/capture_N.png and log every detected object
        if (captureRequested(key)) {
            const int number = captureCount + 1;
            const std::string path = cv::format("%s/capture_%d.png",
                                                DOCS_DIR.c_str(), number);
            if (!cv::imwrite(path, display)) {
                std::cerr << "cannot write " << path << "\n";
                continue;
            }
            captureCount = number;

            int added = 0;
            for (const auto& object : objects) {
                const Detection& d = object->detection();
                if (!d.found) {
                    continue;
                }
                logEntries.push_back(LogEntry{number, object->name,
                                              d.posMm.x, d.posMm.y, d.posMm.z,
                                              d.box.width});
                ++added;
            }
            writeLog(logEntries);
            std::cout << "saved " << path << " (" << added << " objects)\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}