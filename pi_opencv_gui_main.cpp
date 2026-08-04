#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>

using namespace cv;

// =================== Checkpoint / port configuration ===================

// Checkpoint 0 is always the start/finish line. Checkpoints 1..N-1 are
// intermediate track markers, expected to be crossed in ascending order
// before the car reaches the finish line again to complete a lap.
//
// "defaultCheckpointId" is used as a fallback if the connected RP2040 is
// still running the original, unmodified firmware (which doesn't embed an
// ID in its message) - see the firmware patch notes at the bottom of this
// file for the recommended upgrade that embeds the ID directly.
struct CheckpointPort {
    std::string device;
    int defaultCheckpointId;
};

std::vector<CheckpointPort> CHECKPOINTS = {
    {"/dev/ttyACM0", 0}, // start / finish line
    {"/dev/ttyACM1", 1},
    {"/dev/ttyACM2", 2},
    {"/dev/ttyACM3", 3},
};

const int NUM_CHECKPOINTS = static_cast<int>(CHECKPOINTS.size());
const int FINISH_LINE_ID = 0;

// ----- Per-checkpoint region of interest -----
// With one overhead camera seeing the whole track, colour classification
// MUST be confined to the small area around each checkpoint's beam - not
// the whole frame - otherwise a trigger at checkpoint 2 could get matched
// against some other car sitting anywhere else on the track, and two
// checkpoints firing close together would likely both get attributed to
// whichever single car has the most matching pixels overall.
//
// Calibrate these by grabbing one still frame from the overhead camera
// (e.g. `rpicam-still -o track.jpg`), opening it in an image viewer, and
// noting the pixel rectangle around each checkpoint's crossing zone.
// Coordinates are in the same 400x300 space the camera pipeline outputs.
std::map<int, Rect> CHECKPOINT_ROIS = {
    {0, Rect(20,  20,  60, 60)},   // finish line
    {1, Rect(150, 40,  60, 60)},
    {2, Rect(280, 100, 60, 60)},
    {3, Rect(150, 220, 60, 60)},
};

// Clamps a requested ROI to the actual frame bounds, so a slightly
// mis-calibrated rectangle can't crash cv::Mat's operator() with an
// out-of-bounds crop.
Rect clampToFrame(const Rect &roi, const Mat &frame)
{
    Rect bounds(0, 0, frame.cols, frame.rows);
    return roi & bounds; // Rect intersection
}

const int MIN_MATCH_PIXELS = 400;

struct CarProfile {
    std::string name;
    Scalar low;
    Scalar high;
};

std::vector<CarProfile> CAR_PROFILES = {
    {"red_car",        Scalar(0,   120, 70),  Scalar(10,  255, 255)},
    {"blue_car",       Scalar(100, 120, 70),  Scalar(130, 255, 255)},
    {"green_car",      Scalar(35,  50,  50),  Scalar(85,  255, 255)},
    {"brown_car",      Scalar(5,   60,  20),  Scalar(20,  255, 150)},
    {"yellow_highlighter", Scalar(25, 100, 150), Scalar(35, 255, 255)}, // test target
};

const int OPEN_SIZE = 2;
const int CLOSE_SIZE = 4;

// =================== Shared state ===================

std::mutex frameMutex;
Mat latestFrame;
std::atomic<bool> running{true};

struct CarState {
    int expectedNextCheckpoint = 1;   // which checkpoint this car should hit next
    bool sawStart = false;            // whether it has crossed the finish line at least once
    std::chrono::steady_clock::time_point lastFinishTime{};
    double bestLapSeconds = -1;
    double lastLapSeconds = -1;
    int lapCount = 0;
    bool flagged = false;             // true if the most recent event looked like a shortcut/missed checkpoint
};

std::mutex leaderboardMutex;
std::map<std::string, CarState> leaderboard;

// =================== Interactive GUI state ===================

// While false, checkpoint triggers are received and logged but ignored for
// scoring - this is the "race hasn't started" gate controlled by the START
// button drawn in the OpenCV window below.
std::atomic<bool> raceActive{false};

const char *WINDOW_NAME = "Leaderboard";
const Rect START_BUTTON_RECT(20, 20, 160, 50);
const Rect RESET_BUTTON_RECT(200, 20, 160, 50);

// OpenCV mouse callback - checks whether a left-click landed inside either
// button's rectangle and reacts accordingly.
void onMouse(int event, int x, int y, int /*flags*/, void * /*userdata*/)
{
    if (event != EVENT_LBUTTONDOWN) return;

    Point click(x, y);
    if (START_BUTTON_RECT.contains(click)) {
        raceActive = !raceActive;
        std::cout << "[GUI] Race " << (raceActive ? "STARTED" : "STOPPED") << std::endl;
    } else if (RESET_BUTTON_RECT.contains(click)) {
        std::lock_guard<std::mutex> lock(leaderboardMutex);
        leaderboard.clear();
        std::cout << "[GUI] Leaderboard reset" << std::endl;
    }
}

// =================== Serial helpers ===================

int openSerialPort(const std::string &device, speed_t baud)
{
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << ": " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed on " << device << ": " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed on " << device << ": " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

bool readLine(int fd, std::string &outLine)
{
    outLine.clear();
    char ch;
    while (running) {
        ssize_t n = read(fd, &ch, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return false;
        }
        if (n == 0) continue;
        if (ch == '\n') return true;
        if (ch != '\r') outLine.push_back(ch);
    }
    return false;
}

// =================== Trigger parsing ===================

// Matches the lap-completion line from the RP2040 firmware. Checks for
// "Total race time:" (current firmware, using the digital break-beam
// sensor) or "Lap count:" (older ADC-based firmware, kept for backwards
// compatibility) so either version's output is recognised as a trigger.
bool isLapTriggerLine(const std::string &line)
{
    return line.find("Total race time:") != std::string::npos
        || line.find("Lap count:") != std::string::npos;
}

// Looks for a "CP:<id>" prefix, e.g. "CP:2 Lap 1/100 | Total race time: ..."
// or the older "CP:2 Lap count: ...". Only requires the "CP:<id>" prefix
// itself to match, so it works regardless of which message format follows -
// add this prefix to whichever printf() your firmware currently uses.
bool parseEmbeddedCheckpointId(const std::string &line, int &checkpointId)
{
    return sscanf(line.c_str(), "CP:%d", &checkpointId) == 1;
}

// =================== Colour classification ===================

std::string classifyCar(const Mat &bgrFrame, const Rect &roi)
{
    Rect safeRoi = clampToFrame(roi, bgrFrame);
    if (safeRoi.width <= 0 || safeRoi.height <= 0) {
        std::cerr << "[WARN] Checkpoint ROI is entirely outside the frame - check calibration." << std::endl;
        return "";
    }

    Mat cropped = bgrFrame(safeRoi);

    Mat hsv;
    cvtColor(cropped, hsv, COLOR_BGR2HSV);

    std::string bestName;
    int bestCount = 0;

    Mat openElement  = getStructuringElement(MORPH_ELLIPSE, Size(2 * OPEN_SIZE + 1, 2 * OPEN_SIZE + 1));
    Mat closeElement = getStructuringElement(MORPH_ELLIPSE, Size(2 * CLOSE_SIZE + 1, 2 * CLOSE_SIZE + 1));

    for (const auto &car : CAR_PROFILES) {
        Mat mask;
        inRange(hsv, car.low, car.high, mask);
        morphologyEx(mask, mask, MORPH_OPEN, openElement);
        morphologyEx(mask, mask, MORPH_CLOSE, closeElement);

        int count = countNonZero(mask);
        if (count > bestCount) {
            bestCount = count;
            bestName = car.name;
        }
    }

    if (bestCount < MIN_MATCH_PIXELS) return "";
    return bestName;
}

// =================== Leaderboard + shortcut detection ===================

void handleCheckpointEvent(const std::string &carName, int checkpointId)
{
    std::lock_guard<std::mutex> lock(leaderboardMutex);
    CarState &state = leaderboard[carName];

    if (checkpointId == FINISH_LINE_ID) {
        auto now = std::chrono::steady_clock::now();

        bool completedCleanly = state.sawStart && (state.expectedNextCheckpoint == FINISH_LINE_ID);

        if (state.sawStart) {
            double lapSeconds = std::chrono::duration<double>(now - state.lastFinishTime).count();
            state.lapCount++;
            state.lastLapSeconds = lapSeconds;
            if (state.bestLapSeconds < 0 || lapSeconds < state.bestLapSeconds) {
                state.bestLapSeconds = lapSeconds;
            }

            if (completedCleanly) {
                state.flagged = false;
                std::cout << "[LEADERBOARD] " << carName
                          << " completed lap " << state.lapCount
                          << " in " << lapSeconds << "s"
                          << " (best: " << state.bestLapSeconds << "s)" << std::endl;
            } else {
                state.flagged = true;
                std::cout << "[LEADERBOARD][FLAGGED] " << carName
                          << " reached the finish line without hitting checkpoint "
                          << state.expectedNextCheckpoint
                          << " - possible shortcut. Lap " << state.lapCount
                          << " logged as " << lapSeconds << "s but flagged for review." << std::endl;
            }
        } else {
            std::cout << "[LEADERBOARD] " << carName << " crossed the start line" << std::endl;
        }

        state.lastFinishTime = now;
        state.sawStart = true;
        state.expectedNextCheckpoint = (NUM_CHECKPOINTS > 1) ? 1 : FINISH_LINE_ID;
        return;
    }

    // Intermediate checkpoint.
    if (!state.sawStart) {
        // Car showed up mid-track before ever crossing the start line -
        // still record it, but there's no lap in progress to validate against yet.
        std::cout << "[CHECKPOINT] " << carName << " passed checkpoint " << checkpointId
                  << " (no active lap - hasn't crossed start yet)" << std::endl;
        return;
    }

    if (checkpointId == state.expectedNextCheckpoint) {
        std::cout << "[CHECKPOINT] " << carName << " passed checkpoint " << checkpointId
                  << " (on course)" << std::endl;
    } else {
        std::cout << "[CHECKPOINT][FLAGGED] " << carName << " passed checkpoint " << checkpointId
                  << " but checkpoint " << state.expectedNextCheckpoint
                  << " was expected - possible missed checkpoint or shortcut." << std::endl;
    }

    // Advance expectation regardless, so tracking doesn't get permanently
    // stuck if one checkpoint's sensor is flaky - a flag was already logged above.
    state.expectedNextCheckpoint = (checkpointId + 1) % NUM_CHECKPOINTS;
}

// =================== Central event handler (shared by all serial threads) ===================

void processCheckpointEvent(int checkpointId)
{
    if (!raceActive) {
        std::cout << "[IGNORED] Checkpoint " << checkpointId
                   << " triggered but the race hasn't been started (press START)." << std::endl;
        return;
    }

    Mat frameCopy;
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (latestFrame.empty()) return;
        latestFrame.copyTo(frameCopy);
    }

    auto roiIt = CHECKPOINT_ROIS.find(checkpointId);
    if (roiIt == CHECKPOINT_ROIS.end()) {
        std::cerr << "[WARN] No ROI calibrated for checkpoint " << checkpointId
                   << " - add one to CHECKPOINT_ROIS." << std::endl;
        return;
    }

    std::string carName = classifyCar(frameCopy, roiIt->second);
    if (carName.empty()) {
        std::cout << "[WARN] Checkpoint " << checkpointId
                  << " triggered but no car colour matched confidently in its ROI." << std::endl;
        return;
    }

    handleCheckpointEvent(carName, checkpointId);
}

// =================== Leaderboard rendering (OpenCV window, Lab 7 style) ===================

Mat renderLeaderboard()
{
    const int width = 900;
    const int rowHeight = 40;

    // Copy + sort under lock, then release before drawing so we're not
    // holding the mutex while doing (comparatively slow) drawing calls.
    std::vector<std::pair<std::string, CarState>> rows;
    {
        std::lock_guard<std::mutex> lock(leaderboardMutex);
        for (const auto &entry : leaderboard) rows.push_back(entry);
    }
    std::sort(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
        if (a.second.lapCount != b.second.lapCount) return a.second.lapCount > b.second.lapCount;
        double aBest = a.second.bestLapSeconds < 0 ? 1e9 : a.second.bestLapSeconds;
        double bBest = b.second.bestLapSeconds < 0 ? 1e9 : b.second.bestLapSeconds;
        return aBest < bBest;
    });

    int height = 200 + static_cast<int>(rows.size()) * rowHeight;
    Mat img(std::max(height, 260), width, CV_8UC3, Scalar(20, 20, 16)); // dark BGR background

    // ----- Title -----
    putText(img, "RACE LEADERBOARD", Point(20, 110), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(240, 240, 240), 2);

    // ----- Buttons -----
    Scalar startColor = raceActive ? Scalar(60, 170, 60) : Scalar(60, 60, 200); // green if active, red if stopped
    rectangle(img, START_BUTTON_RECT, startColor, FILLED);
    rectangle(img, START_BUTTON_RECT, Scalar(255, 255, 255), 1);
    putText(img, raceActive ? "STOP" : "START", Point(START_BUTTON_RECT.x + 28, START_BUTTON_RECT.y + 33),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

    rectangle(img, RESET_BUTTON_RECT, Scalar(90, 90, 90), FILLED);
    rectangle(img, RESET_BUTTON_RECT, Scalar(255, 255, 255), 1);
    putText(img, "RESET", Point(RESET_BUTTON_RECT.x + 28, RESET_BUTTON_RECT.y + 33),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

    // ----- Status -----
    std::string status = raceActive ? "RACE ACTIVE" : "RACE STOPPED - checkpoint triggers are ignored";
    putText(img, status, Point(380, 45), FONT_HERSHEY_SIMPLEX, 0.6,
            raceActive ? Scalar(120, 255, 120) : Scalar(120, 120, 255), 1);

    // ----- Table header -----
    int headerY = 150;
    putText(img, "Rank", Point(20, headerY),  FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    putText(img, "Car",  Point(100, headerY), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    putText(img, "Laps", Point(320, headerY), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    putText(img, "Best Lap", Point(420, headerY), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    putText(img, "Last Lap", Point(580, headerY), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    putText(img, "Status",   Point(740, headerY), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    line(img, Point(20, headerY + 10), Point(width - 20, headerY + 10), Scalar(90, 90, 90), 1);

    // ----- Rows -----
    int y = headerY + 40;
    int rank = 1;
    for (const auto &entry : rows) {
        const std::string &name = entry.first;
        const CarState &s = entry.second;

        Scalar rowColor = s.flagged ? Scalar(80, 80, 255)
                         : (rank == 1 ? Scalar(140, 255, 140) : Scalar(230, 230, 230));

        putText(img, std::to_string(rank), Point(20, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);
        putText(img, name, Point(100, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);
        putText(img, std::to_string(s.lapCount), Point(320, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);

        char bestBuf[32];
        if (s.bestLapSeconds >= 0) snprintf(bestBuf, sizeof(bestBuf), "%.3fs", s.bestLapSeconds);
        else snprintf(bestBuf, sizeof(bestBuf), "--");
        putText(img, bestBuf, Point(420, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);

        char lastBuf[32];
        if (s.lastLapSeconds >= 0) snprintf(lastBuf, sizeof(lastBuf), "%.3fs", s.lastLapSeconds);
        else snprintf(lastBuf, sizeof(lastBuf), "--");
        putText(img, lastBuf, Point(580, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);

        putText(img, s.flagged ? "FLAGGED" : "OK", Point(740, y), FONT_HERSHEY_SIMPLEX, 0.6, rowColor, 1);

        y += rowHeight;
        rank++;
    }

    if (rows.empty()) {
        putText(img, "Waiting for the first checkpoint crossing...", Point(20, y),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(180, 180, 180), 1);
    }

    return img;
}

// =================== Camera thread ===================

void cameraThreadFunc()
{
    std::string pipeline = "libcamerasrc"
        " ! video/x-raw, width=800, height=600"
        " ! videoconvert"
        " ! videoscale"
        " ! video/x-raw, width=400, height=300"
        " ! videoflip method=rotate-180"
        " ! appsink drop=true max_buffers=2";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Could not open camera." << std::endl;
        running = false;
        return;
    }

    Mat frame;
    while (running) {
        if (!cap.read(frame)) {
            std::cerr << "Could not read a frame." << std::endl;
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            frame.copyTo(latestFrame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    cap.release();
}

// =================== One serial listener thread per checkpoint board ===================

void serialThreadFunc(CheckpointPort port)
{
    int fd = openSerialPort(port.device, B115200);
    if (fd < 0) {
        std::cerr << "Skipping checkpoint " << port.defaultCheckpointId
                   << " - could not open " << port.device << std::endl;
        return;
    }

    std::cout << "Listening for checkpoint " << port.defaultCheckpointId
               << " on " << port.device << std::endl;

    std::string line;
    while (running) {
        if (!readLine(fd, line)) {
            if (!running) break;
            std::cerr << port.device << ": serial read error, retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        if (line.empty()) continue;

        int checkpointId;
        if (parseEmbeddedCheckpointId(line, checkpointId)) {
            // Upgraded firmware sent its own ID - trust it.
        } else if (isLapTriggerLine(line)) {
            // Unmodified firmware - fall back to the port's configured ID.
            checkpointId = port.defaultCheckpointId;
        } else {
            continue; // diagnostic line, e.g. "No car." - ignore
        }

        std::cout << "[TRIGGER] " << port.device << ": " << line << std::endl;
        processCheckpointEvent(checkpointId);
    }

    close(fd);
}

// =================== Manual trigger mode (no RP2040 needed) ===================

// Lets you test the camera capture + colour classification + leaderboard
// pipeline without any break-beam hardware attached. Type a checkpoint ID
// and press Enter; it calls the exact same processCheckpointEvent() that a
// real serial trigger would, so everything downstream of "a trigger fired"
// is tested for real - only the trigger source itself is swapped out.
void manualTriggerThreadFunc()
{
    std::cout << "\n[MANUAL MODE] No serial ports opened - simulating triggers from the keyboard.\n"
              << "Type a checkpoint ID (0 = finish line, 1.." << (NUM_CHECKPOINTS - 1)
              << " = intermediate) and press Enter. Ctrl+D to quit.\n" << std::endl;

    std::string input;
    while (running && std::getline(std::cin, input)) {
        if (input.empty()) continue;
        try {
            int checkpointId = std::stoi(input);
            std::cout << "[MANUAL TRIGGER] Simulating checkpoint " << checkpointId << std::endl;
            processCheckpointEvent(checkpointId);
        } catch (const std::exception &) {
            std::cout << "[MANUAL] Please enter a valid integer checkpoint ID." << std::endl;
        }
    }
    running = false;
}

// =================== Main ===================

int main(int argc, char *argv[])
{
    bool manualMode = false;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--manual" || std::string(argv[i]) == "-m") {
            manualMode = true;
        }
    }

    std::cout << "Starting multi-checkpoint tracker with " << NUM_CHECKPOINTS << " checkpoints." << std::endl;
    std::cout << "Click START in the window (or press S) to begin scoring checkpoint triggers." << std::endl;

    // Camera and checkpoint listening run in the background, same as before.
    std::thread camThread(cameraThreadFunc);

    std::vector<std::thread> triggerThreads;
    if (manualMode) {
        triggerThreads.emplace_back(manualTriggerThreadFunc);
    } else {
        for (const auto &port : CHECKPOINTS) {
            triggerThreads.emplace_back(serialThreadFunc, port);
        }
    }

    // ----- GUI runs on the main thread, Lab 7 style -----
    namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
    setMouseCallback(WINDOW_NAME, onMouse);
    namedWindow("Camera + ROIs", WINDOW_AUTOSIZE); // calibration aid

    while (running) {
        Mat img = renderLeaderboard();
        imshow(WINDOW_NAME, img);

        // Debug view: overhead camera feed with each checkpoint's ROI drawn
        // on it, so you can visually confirm the rectangles in
        // CHECKPOINT_ROIS actually line up with where the beams are.
        {
            Mat debugFrame;
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                if (!latestFrame.empty()) latestFrame.copyTo(debugFrame);
            }
            if (!debugFrame.empty()) {
                for (const auto &entry : CHECKPOINT_ROIS) {
                    Rect safeRoi = clampToFrame(entry.second, debugFrame);
                    rectangle(debugFrame, safeRoi, Scalar(0, 255, 255), 2);
                    putText(debugFrame, "CP" + std::to_string(entry.first),
                            Point(safeRoi.x, std::max(0, safeRoi.y - 5)),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
                }
                imshow("Camera + ROIs", debugFrame);
            }
        }

        int key = waitKey(100);
        if (key == 27) {              // ESC - quit
            running = false;
        } else if (key == 's' || key == 'S') { // keyboard shortcut for the START/STOP button
            raceActive = !raceActive;
            std::cout << "[GUI] Race " << (raceActive ? "STARTED" : "STOPPED") << std::endl;
        } else if (key == 'r' || key == 'R') { // keyboard shortcut for RESET
            std::lock_guard<std::mutex> lock(leaderboardMutex);
            leaderboard.clear();
            std::cout << "[GUI] Leaderboard reset" << std::endl;
        }
    }

    if (manualMode) {
        std::cout << "Press Enter in the terminal to finish shutting down the manual trigger thread..." << std::endl;
    }

    camThread.join();
    for (auto &t : triggerThreads) t.join();

    return 0;
}

/*
=================== Firmware patch (recommended) ===================

The stock CC3501-Break_Beam_Sensor firmware has no checkpoint ID field, so
by default this program falls back to identifying checkpoints by which
serial port they're plugged into (CHECKPOINTS table above). That works,
but USB enumeration order (/dev/ttyACM0, ACM1, ...) is not guaranteed to
stay the same across reboots or reconnections, especially with 3+ boards.

For a reliable multi-board setup, add a per-board ID constant near the top
of the RP2040's main.cpp:

    #define CHECKPOINT_ID 2   // set uniquely per board before flashing

...and update the lap-detection branch in detect_car() to both (a) print
the ID and (b) fire before the buzzer, not after, so latency stays low:

    if (is_same_lap == 0) {
        lap_times[lap_count] = time_between_readings;
        lap_count += 1;
        printf("CP:%d Lap count: %3d | Total time to get lap: %5d ms (%2d sec : %3d ms)\n",
               CHECKPOINT_ID, lap_count, time_between_readings,
               time_between_readings / 1000, time_between_readings % 1000);
        is_same_lap = 1;
    }
    activate_buzzer(timer_interval);

With that change, parseEmbeddedCheckpointId() above will pick up the ID
directly from the message, and which physical /dev/ttyACM* port a board
happens to land on no longer matters.

Alternative to firmware changes: pin each board to a stable device path
using a udev rule keyed on its USB serial number (e.g.
/etc/udev/rules.d/99-checkpoints.rules mapping each board's serial to
/dev/checkpoint0, /dev/checkpoint1, ...), then use those stable paths in
the CHECKPOINTS table instead of relying on ACM numbering.
*/