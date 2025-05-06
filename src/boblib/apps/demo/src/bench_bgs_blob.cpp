#include <iostream>
#include <string>
#include <algorithm>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "./../../api/bgs/bgs.hpp"
#include "./../../api/blobs/connectedBlobDetection.hpp"
#include "./../../api/utils/profiler.hpp"

/////////////////////////////////////////////////////////////
// Default parameters
int blur_radius{3};
bool applyGreyscale{true};
bool applyNoiseReduction{false};
int sensitivity{1};

/////////////////////////////////////////////////////////////
// Background subtractor to use
enum BGSType
{
    Vibe,
    WMV
};
std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};

std::unique_ptr<boblib::bgs::CoreBgs> bgs_ptr_{nullptr};
std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

/////////////////////////////////////////////////////////////
// Blob Detector
boblib::blobs::ConnectedBlobDetection blobDetector;

/////////////////////////////////////////////////////////////
// Function Definitions
std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BGSType _type);
inline void appyPreProcess(const boblib::base::Image &input, boblib::base::Image &output);
inline void appyBGS(const boblib::base::Image &input, boblib::base::Image &output);
inline void drawBboxes(std::vector<cv::Rect> &bboxes, boblib::base::Image &frame);
inline void findBlobs(const boblib::base::Image &image, std::vector<cv::Rect> &blobs);
inline void drawBboxes(std::vector<cv::Rect> &keypoints, const cv::Mat &frame);
inline void outputBoundingBoxes(std::vector<cv::Rect> &bboxes);
int getIntArg(std::string arg);

/////////////////////////////////////////////////////////////
// Main entry point for demo
int main(int argc, const char **argv)
{
    std::string videoFile{"mike-drone.mp4"};

    std::cout << "Available number of concurrent threads = " << std::thread::hardware_concurrency() << std::endl;

    bgsPtr = createBGS(BGSType::Vibe);
    cv::VideoCapture cap;

    if (argc > 1)
    {
        int camNum = getIntArg(argv[1]);
        if (camNum >= 0)
        {
            cap.open(camNum);
        }
        else
        {
            cap.open(argv[1]);
        }
    }
    else
    {
        cap.open(videoFile);
    }

    if (!cap.isOpened())
    {
        std::cout << "***Could not initialize capturing...***" << std::endl;
        return -1;
    }

    double frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Capture size: " << (int)frameWidth << " x " << (int)frameHeight << std::endl;

    cv::namedWindow("BGS Demo", 0);
    cv::namedWindow("Live Video", 0);

    cv::Mat frame;

    cap.read(frame);
    if (frame.type() != CV_8UC3)
    {
        std::cout << "Image type not supported" << std::endl;
        return -1;
    }
    boblib::utils::Profiler profiler;

    boblib::base::Image bgsMaskImg;

    std::vector<cv::Rect> bboxes;
    bool pause = false;
    std::cout << "Enter loop" << std::endl;
    while (true)
    {
        if (!pause)
        {
            cap.read(frame);
            if (frame.empty())
            {
                std::cout << "No image" << std::endl;
                break;
            }
            boblib::base::Image img_input(frame);
            boblib::base::Image processedFrame;

            profiler.start(0, "Preprocess");
            if (applyGreyscale)
            {
                img_input.convertColorTo(processedFrame, cv::COLOR_RGB2GRAY);
            }
            else
            {
                processedFrame = img_input;
            }
            if (applyNoiseReduction)
            {
                processedFrame.medianBlur(blur_radius);
            }
            profiler.stop(0);
            profiler.start(1, "BGS");
            bgsPtr->apply(processedFrame, bgsMaskImg);
            profiler.stop(1);
            profiler.start(2, "Blob");
            blobDetector.detect(bgsMaskImg, bboxes);
            profiler.stop(2);
            drawBboxes(bboxes, bgsMaskImg);
            drawBboxes(bboxes, img_input);

            std::string profiler_report;
            if (profiler.report_if_greater(5.0, profiler_report))
            {
                std::cout << profiler_report << std::endl;
            }

            cv::imshow("BGS Demo", bgsMaskImg.toMat());
            cv::resizeWindow("BGS Demo", 1024, 1024);
            cv::imshow("Live Video", img_input.toMat());
            cv::resizeWindow("Live Video", 1024, 1024);
        }
        char key = (char)cv::waitKey(1);
        if (key == 27)
        {
            std::cout << "Escape key pressed" << std::endl;
            break;
        }
        else if (key == 32)
        {
            pause = !pause;
            outputBoundingBoxes(bboxes);
        }
        else if (key == '+')
        {
            auto params = (boblib::bgs::WMVParams&)(bgsPtr->get_parameters());
            float threshold = params.get_threshold();
            std::cout << "Got threshold: " << threshold << std::endl;
            params.set_threshold(threshold + 5);
        }
        else if (key == '-')
        {
            auto params = (boblib::bgs::WMVParams&)(bgsPtr->get_parameters());
            float threshold = params.get_threshold();
            std::cout << "Got threshold: " << threshold << std::endl;
            params.set_threshold(threshold - 5);
        }
    }
    std::cout << "Exit loop\n"
              << std::endl;

    cap.release();

    cv::destroyAllWindows();

    return 0;
}

std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BGSType _type)
{
    switch (_type)
    {
    case BGSType::Vibe:
        return std::make_unique<boblib::bgs::Vibe>(boblib::bgs::VibeParams(50, 24, 1, 8));
    case BGSType::WMV:
        return std::make_unique<boblib::bgs::WeightedMovingVariance>();
    default:
        return std::make_unique<boblib::bgs::WeightedMovingVariance>();
    }
}

inline void outputBoundingBoxes(std::vector<cv::Rect> &bboxes)
{
    std::cout << "Bounding boxes" << std::endl;
    for (auto bb : bboxes)
    {
        std::cout << bb << std::endl;
    }
}

inline void drawBboxes(std::vector<cv::Rect> &bboxes, boblib::base::Image &frame)
{
    auto &frameMat = frame.toMat();
    for (auto bb : bboxes)
    {
        cv::rectangle(frameMat, bb, cv::Scalar(255, 0, 255), 2);
    }
}

int getIntArg(std::string arg)
{
    std::size_t pos{};
    try
    {
        const int argNum{std::stoi(arg, &pos)};
        return pos == arg.size() ? argNum : -1;
    }
    catch (std::exception const &ex)
    {
        return -1;
    }
}