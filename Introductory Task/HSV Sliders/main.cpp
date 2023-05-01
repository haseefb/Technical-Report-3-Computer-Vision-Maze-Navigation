#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// Define the mouse callback function
void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Get the color from the mouse position
        Mat hsv = *((Mat*)userdata);
        Vec3b color = hsv.at<Vec3b>(y, x);

        // Set the values of the trackbars to the color values
        setTrackbarPos("Hue Min", "Trackbar", color[0] - 10);
        setTrackbarPos("Sat Min", "Trackbar", color[1] - 10);
        setTrackbarPos("Val Min", "Trackbar", color[2] - 10);
        setTrackbarPos("Hue Max", "Trackbar", color[0] + 10);
        setTrackbarPos("Sat Max", "Trackbar", color[1] + 10);
        setTrackbarPos("Val Max", "Trackbar", color[2] + 10);
    }
}

int main(int argc, char** argv)
{
    // Create a window for the trackbar
    namedWindow("Trackbar", WINDOW_NORMAL);

    // Initialize the values for the trackbar
    int hmin = 0, smin = 0, vmin = 0;
    int hmax = 179, smax = 255, vmax = 255;

    // Create the trackbars
    createTrackbar("Hue Min", "Trackbar", &hmin, 179);
    createTrackbar("Sat Min", "Trackbar", &smin, 255);
    createTrackbar("Val Min", "Trackbar", &vmin, 255);
    createTrackbar("Hue Max", "Trackbar", &hmax, 179);
    createTrackbar("Sat Max", "Trackbar", &smax, 255);
    createTrackbar("Val Max", "Trackbar", &vmax, 255);

    // Create a window to display the image
    namedWindow("Image", WINDOW_NORMAL);

    // Load the image
    Mat image = imread("img.bmp");

    // Convert the image to HSV
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);

    // Set the mouse callback function
    setMouseCallback("Image", mouseCallback, &hsv);

    while (true)
    {
        // Get the values from the trackbars
        int h1 = hmin, s1 = smin, v1 = vmin;
        int h2 = hmax, s2 = smax, v2 = vmax;

        // Create a mask based on the values from the trackbars
        Mat mask;
        inRange(hsv, Scalar(h1, s1, v1), Scalar(h2, s2, v2), mask);

        // Apply the mask to the original image
        Mat result;
        bitwise_and(image, image, result, mask);

        // Show the original image and the result
        imshow("Image", result);

        // Wait for a key press
        int key = waitKey(1);

        // If the user presses the 'q' key, quit the program
        if (key == 'q')
        {
            break;
        }
    }

    return 0;
}
