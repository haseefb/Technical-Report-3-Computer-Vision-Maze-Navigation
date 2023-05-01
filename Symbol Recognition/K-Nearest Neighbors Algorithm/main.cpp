#include <stdio.h>
#include "opencv_aee.hpp"
#include "main.hpp"
#include "pi2c.h"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace cv::ml;

class MyConstructor {
public:
    std::vector<std::string> folderPaths;
    std::vector<int> Names;

    MyConstructor() {
        folderPaths.push_back("/home/pi/Desktop/OpenCV-Template/star");
        folderPaths.push_back("/home/pi/Desktop/OpenCV-Template/circles");
        folderPaths.push_back("/home/pi/Desktop/OpenCV-Template/triangle");
        folderPaths.push_back("/home/pi/Desktop/OpenCV-Template/umbrella");

        int arr[] = {0,1,2,3};
        Names = std::vector<int>(arr, arr + sizeof(arr) / sizeof(int));
    }
};

void setup(void)
{
    setupCamera(320, 240);
}

int main(int argc, char** argv)
{
    // Load training images
    MyConstructor myObject;
    std::vector<std::string> folderPaths = myObject.folderPaths;
    std::vector<int> Names = myObject.Names;
    Mat trainingData;
    Mat names;

    for (int i = 0; i < folderPaths.size(); i++) {
        // Read all images from the current folder
        std::vector<cv::String> filenames;
        glob(folderPaths[i] + "/*.png", filenames, false);

        // Process each image in the folder
        for (int j = 0; j < filenames.size(); j++) {
            // Reads the training image
            Mat images = imread(filenames[j], IMREAD_GRAYSCALE);

            // Check that that all the images have loaded onto the matrix
            if (images.empty()) {
                printf("Error: image '%s' could not be loaded\n", filenames[j].c_str());
                continue; // Doesn't stop the program even if error is printed
            }

            // Resizes the training image
            Size imageSize(100, 100);
            resize(images, images, imageSize);

            // Add image to training samples and labels
            trainingData.push_back(images.clone().reshape(1, 1));
            names.push_back(Names[i]);
            }
        }
        // Create KNN model
        Ptr<KNearest> knn = KNearest::create();
        trainingData.convertTo(trainingData, CV_32F);

        // Train KNN model
        knn->train(trainingData, ROW_SAMPLE, names);

        setup();
        namedWindow("Camera Feed");

        while (true) {
            // Capture frame
            Mat frame;
            frame = captureFrame();

            // Convert frame to grayscale
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            // Resize frame to match training images
            Size imageSize(100, 100);
            resize(gray, gray, imageSize);

            // Preprocess frame and ensure data type is the same as training data
            Mat testSample = gray.clone().reshape(1, 1);
            testSample.convertTo(testSample, CV_32F);

            // KNN model used to identify which symbol is in the frame
            Mat results, neighborResponses, dists;
            int k = 1; // Number of neighbors to use for prediction
            knn->findNearest(testSample, k, results, neighborResponses, dists);

            int names = (int)results.at<float>(0, 0);
            std::string labelString;

            switch (names) {
            case 0:
                labelString = "Star";
                break;
            case 1:
                labelString = "Circle";
                break;
            case 2:
                labelString = "Triangle";
                break;
            case 3:
                labelString = "Umbrella";
                break;
            default:
                labelString = "Unknown";
                break;
            }
        //result displayed as text on window
        putText(frame, labelString, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

        // Display frame on screen
        imshow("Camera Feed", frame);


        if (waitKey(30) == 'q') {
            break;
        }
    }

    // Cleanup
    destroyAllWindows();

    return 0;
}

