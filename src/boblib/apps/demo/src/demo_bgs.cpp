#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>

class SiameseTracker
{
public:
    // Constructor to load the ONNX models
    SiameseTracker(const std::string &templateModelPath, const std::string &modelPath)
    {
        // Load the template and tracking models
        templateNet = cv::dnn::readNetFromONNX(templateModelPath);
        modelNet = cv::dnn::readNetFromONNX(modelPath);

        // Set backend to CUDA (if available) for both networks
        templateNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        templateNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        modelNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        modelNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }

    // Initialize the tracker with the first frame and bounding box
    void initialize(const cv::Mat &frame, const cv::Rect2d &bbox)
    {
        // Crop the region of interest (ROI) around the bounding box
        cv::Mat roi = frame(bbox);

        // Preprocess the ROI to create the input blob
        cv::Mat blob = cv::dnn::blobFromImage(roi, 1.0, cv::Size(127, 127), cv::Scalar(), true, false);

        // Pass the blob to the template network (use input name "1")
        templateNet.setInput(blob, "1");

        // Forward pass to get the template features (likely kernels for regression and classification)
        std::vector<cv::Mat> outputs;
        std::vector<std::string> outputNames = {"112", "114"};
        templateNet.forward(outputs, outputNames);

        // Store the extracted template features (kernels)
        kernelReg = outputs[0]; // Regression kernel from template (output 112)
        kernelCls = outputs[1]; // Classification kernel from template (output 114)
    }

    // Track the object in a new frame and return the updated bounding box
    cv::Rect2d track(const cv::Mat &frame, const cv::Rect2d &prevBbox)
    {
        // Define the search region around the previous bounding box (larger than bbox)
        cv::Rect2d searchRegion(prevBbox.x - prevBbox.width / 2, prevBbox.y - prevBbox.height / 2,
                                prevBbox.width * 2, prevBbox.height * 2);
        searchRegion &= cv::Rect2d(0, 0, frame.cols, frame.rows); // Ensure it's within frame bounds

        // Crop the search region from the current frame
        cv::Mat searchRoi = frame(searchRegion);

        // Preprocess the search region to create the input blob
        cv::Mat blob = cv::dnn::blobFromImage(searchRoi, 1.0, cv::Size(255, 255), cv::Scalar(), true, false);

        // Set the inputs for the model (search region and template kernels)
        modelNet.setInput(blob, "1");      // Search region as input
        modelNet.setInput(kernelReg, "2"); // Regression kernel
        modelNet.setInput(kernelCls, "3"); // Classification kernel

        // Perform the forward pass to get the outputs
        std::vector<cv::Mat> modelOutputs;
        std::vector<std::string> modelOutputNames = {"125", "132"}; // Output names from the model
        modelNet.forward(modelOutputs, modelOutputNames);

        cv::Mat regOutput = modelOutputs[0]; // Regression output for bounding box
        cv::Mat clsOutput = modelOutputs[1]; // Classification score output

        // 1. Parse the classification score (use max classification score to find the best position)
        cv::Point maxLoc;
        double maxScore;
        cv::minMaxLoc(clsOutput, nullptr, &maxScore, nullptr, &maxLoc);

        // 2. Parse the regression output to update the bounding box
        double dx = regOutput.at<float>(0);
        double dy = regOutput.at<float>(1);
        double dw = regOutput.at<float>(2);
        double dh = regOutput.at<float>(3);

        // Update the bounding box based on the regression values
        cv::Rect2d newBbox = prevBbox;
        newBbox.x += dx * newBbox.width;
        newBbox.y += dy * newBbox.height;
        newBbox.width *= std::exp(dw);
        newBbox.height *= std::exp(dh);

        // Return the updated bounding box
        return newBbox;
    }

private:
    cv::dnn::Net templateNet; // Network to extract template features
    cv::dnn::Net modelNet;    // Network to perform the actual tracking
    cv::Mat kernelReg;        // Regression kernel from the template model (output 112)
    cv::Mat kernelCls;        // Classification kernel from the template model (output 114)
};

int main(int, char **argv)
{
    // Open the default camera (usually the webcam)
    std::cout << "Connecting to: " << argv[1] << std::endl;
    cv::VideoCapture cap(argv[1]);

    // Check if the webcam opened successfully
    if (!cap.isOpened())
    {
        std::cout << "Error opening webcam." << std::endl;
        return -1;
    }

    // Create a window
    cv::namedWindow("Webcam Feed", cv::WINDOW_NORMAL);

    // Read a frame to let the user select the bounding box
    cv::Mat frame;
    cv::Rect bbox;
    while (true)
    {
        cap.read(frame);
        if (frame.empty())
        {
            std::cout << "Error reading frame from webcam." << std::endl;
            return -1;
        }
        cv::imshow("Webcam Feed", frame);

        if (cv::waitKey(30) == ' ')
        {
            // Let the user select a bounding box around the object to track
            bbox = cv::selectROI("Webcam Feed", frame, false, false);
            if (bbox.width == 0 || bbox.height == 0)
            {
                std::cout << "No bounding box selected." << std::endl;
                continue;
            }
            break;
        }
    }

    // Initialize the CSRT tracker
    cv::Ptr<cv::TrackerCSRT> tracker = cv::TrackerCSRT::create();

    // Initialize the tracker on the first frame and the selected bounding box
    tracker->init(frame, bbox);

    while (true)
    {
        // Capture a new frame
        cap.read(frame);
        if (frame.empty())
        {
            std::cout << "Error reading frame from webcam." << std::endl;
            break;
        }

        // Update the tracker with the new frame and the existing bounding box
        bool trackingSuccess = tracker->update(frame, bbox);

        // Draw the bounding box if tracking was successful
        if (trackingSuccess)
        {
            cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
        }
        else
        {
            cv::putText(frame, "Tracking failure detected", cv::Point(100, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
        }

        // Display the result
        cv::imshow("Webcam Feed", frame);

        // Exit if 'q' is pressed
        if (cv::waitKey(30) == 'q')
        {
            break;
        }
    }

    // Release the webcam and destroy all windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
