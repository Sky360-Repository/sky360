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
std::unique_ptr<sky360lib::bgs::CoreBgs> bgsPtr{nullptr};

/////////////////////////////////////////////////////////////
// Blob Detector
sky360lib::blobs::ConnectedBlobDetection blobDetector;

/////////////////////////////////////////////////////////////
// Function Definitions
std::unique_ptr<sky360lib::bgs::CoreBgs> createBGS(BGSType _type);
inline void appyPreProcess(const cv::Mat &input, cv::Mat &output);
inline void appyBGS(const cv::Mat &input, cv::Mat &output);
inline void drawBboxes(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &frame);
inline void findBlobs(const cv::Mat &image, std::vector<cv::Rect> &blobs);
inline void drawBboxes(std::vector<cv::Rect> &keypoints, const cv::Mat &frame);
inline void outputBoundingBoxes(std::vector<cv::Rect> &bboxes);
int getIntArg(std::string arg);

/////////////////////////////////////////////////////////////
// Main entry point for demo
int main(int argc, const char **argv)
{
    std::string videoFile{"Dahua-20220901-184734.mp4"};
    // std::string videoFile{"birds_and_plane.mp4"};
    // std::string videoFile{"brad_drone_1.mp4"};

    // Setting some initial configurations
    cv::ocl::setUseOpenCL(true);
    if (cv::ocl::haveOpenCL())
    {
        std::cout << "Has OpenCL support, using: " << (cv::ocl::useOpenCL() ? "Yes" : "No") << std::endl;
    }

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

    // int camNum = std::stoi(argv[1]);
    // cap.open(camNum);
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

    cv::Mat frame, frame16, processedFrame;
    long numFrames{0};
    long totalNumFrames{0};
    double totalTime{0.0};
    double totalProcessedTime{0.0};
    double totalMeanProcessedTime{0.0};

    cap.read(frame);
    if (frame.type() != CV_8UC3)
    {
        std::cout << "Image type not supported" << std::endl;
        return -1;
    }
    cv::Mat bgsMask{frame.size(), CV_8UC1};

    std::vector<cv::Rect> bboxes;
    bool pause = false;
    std::cout << "Enter loop" << std::endl;
    while (true)
    {
        auto startFrameTime = std::chrono::high_resolution_clock::now();
        if (!pause)
        {
            auto startProcessedTime = std::chrono::high_resolution_clock::now();
            cap.read(frame);
            if (frame.empty())
            {
                std::cout << "No image" << std::endl;
                break;
            }
            frame.convertTo(frame16, CV_16UC3, 256.0f);
            appyPreProcess(frame16, processedFrame);
            appyBGS(processedFrame, bgsMask);
            findBlobs(bgsMask, bboxes);
            auto endProcessedTime = std::chrono::high_resolution_clock::now();
            drawBboxes(bboxes, bgsMask);
            drawBboxes(bboxes, frame);
            ++numFrames;
            totalProcessedTime += std::chrono::duration_cast<std::chrono::nanoseconds>(endProcessedTime - startProcessedTime).count() * 1e-9;
            totalMeanProcessedTime += std::chrono::duration_cast<std::chrono::nanoseconds>(endProcessedTime - startProcessedTime).count() * 1e-9;
            ++totalNumFrames;
            cv::imshow("BGS Demo", bgsMask);
            cv::resizeWindow("BGS Demo", 1024, 1024);
            cv::imshow("Live Video", frame);
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
            auto params = (sky360lib::bgs::WMVParams&)(bgsPtr->get_parameters());
            float threshold = params.get_threshold();
            std::cout << "Got threshold: " << threshold << std::endl;
            params.set_threshold(threshold + 5);
        }
        else if (key == '-')
        {
            auto params = (sky360lib::bgs::WMVParams&)(bgsPtr->get_parameters());
            float threshold = params.get_threshold();
            std::cout << "Got threshold: " << threshold << std::endl;
            params.set_threshold(threshold - 5);
        }
        auto endFrameTime = std::chrono::high_resolution_clock::now();
        totalTime += std::chrono::duration_cast<std::chrono::nanoseconds>(endFrameTime - startFrameTime).count() * 1e-9;
        if (totalTime > 2.0)
        {
            std::cout << "Framerate: " << (numFrames / totalProcessedTime) << " fps" << std::endl;
            totalTime = 0.0;
            totalProcessedTime = 0.0;
            numFrames = 0;
        }
    }
    std::cout << "Exit loop\n"
              << std::endl;
    std::cout << std::endl
              << "Mean Framerate: " << (totalNumFrames / totalMeanProcessedTime) << " fps" << std::endl;

    cap.release();

    cv::destroyAllWindows();

    return 0;
}

std::unique_ptr<sky360lib::bgs::CoreBgs> createBGS(BGSType _type)
{
    switch (_type)
    {
    case BGSType::Vibe:
        return std::make_unique<sky360lib::bgs::Vibe>(sky360lib::bgs::VibeParams(50, 24, 1, 8));
    case BGSType::WMV:
        return std::make_unique<sky360lib::bgs::WeightedMovingVariance>();
    default:
        return std::make_unique<sky360lib::bgs::WeightedMovingVariance>();
    }
}

// Do image pre-processing
inline void appyPreProcess(const cv::Mat &input, cv::Mat &output)
{
    cv::Mat tmpFrame;

    if (applyGreyscale)
        cv::cvtColor(input, tmpFrame, cv::COLOR_RGB2GRAY);
    else
        tmpFrame = input;
    if (applyNoiseReduction)
        cv::GaussianBlur(tmpFrame, output, cv::Size(blur_radius, blur_radius), 0);
    else
        output = tmpFrame;
}

// Apply background subtraction
inline void appyBGS(const cv::Mat &input, cv::Mat &output)
{
    bgsPtr->apply(input, output);
}

inline void outputBoundingBoxes(std::vector<cv::Rect> &bboxes)
{
    std::cout << "Bounding boxes" << std::endl;
    for (auto bb : bboxes)
    {
        std::cout << bb << std::endl;
    }
}

inline void drawBboxes(std::vector<cv::Rect> &bboxes, const cv::Mat &frame)
{
    for (auto bb : bboxes)
    {
        cv::rectangle(frame, bb, cv::Scalar(255, 0, 255), 2);
    }
}

// Finds the connected components in the image and returns a list of bounding boxes
inline void findBlobs(const cv::Mat &image, std::vector<cv::Rect> &blobs)
{
    blobDetector.detect(image, blobs);
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