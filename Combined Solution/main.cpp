#include <stdio.h>
#include "opencv_aee.hpp"
#include "main.hpp"
#include "pi2c.h"
#include <thread> // for std::this_thread::sleep_for()
#include <vector>
using namespace cv;


bool isCircle(const std::vector<cv::Point>& poly)
{
    if (poly.size() < 5)
        return false; // Check if poly has at least five points
    cv::RotatedRect rect = cv::fitEllipse(poly); // Fit an ellipse to the polygon
    double poly_area = cv::contourArea(poly);
    double ellipse_area = rect.size.width * rect.size.height * CV_PI / 4.0;

    // Check if the polygon is approximately circular
    return poly_area / ellipse_area >= 0.9;
}


void setup(void)
{
    setupCamera(320, 240);
}

void SendtoArduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
    Pi2c car(0x04);
    char data[6];
    data[0] = (leftMotor_speed >> 8) & 0xFF;
    data[1] = leftMotor_speed & 0xFF;
    data[2] = (rightMotor_speed >> 8) & 0xFF;
    data[3] = rightMotor_speed & 0xFF;
    data[4] = (servoAngle >> 8) & 0xFF;
    data[5] = servoAngle & 0xFF;
    car.i2cWrite(data, 6);
}

int symbol(cv::Mat frame)
{
    cv::Mat hsv_frame;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV); // Convert the image to HSV color space

// Define the lower and upper bounds of the pink color in HSV color space
    cv::Scalar lower_pink(140, 50, 50);
    cv::Scalar upper_pink(180, 255, 255);

    cv::Mat pink_mask;
    cv::inRange(hsv_frame, lower_pink, upper_pink, pink_mask); // Create a binary mask of the pink pixels

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(pink_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Find the contours of the binary image

    int largest_contour_index = -1;
    double largest_contour_area = 0.0;

    for (int i = 0; i < contours.size(); i++)   // Iterate over all the contours to find the largest one
    {
        double area = cv::contourArea(contours[i]);
        if (area > largest_contour_area)
        {
            largest_contour_index = i;
            largest_contour_area = area;
        }
    }

    if (largest_contour_index >= 0)   // If a contour was found
    {
        std::vector<cv::Point> largest_contour_poly;
        cv::approxPolyDP(contours[largest_contour_index], largest_contour_poly, 0.01 * cv::arcLength(contours[largest_contour_index], true), true); // Approximate the largest contour to a polygon

        int num_vertices = largest_contour_poly.size();

        int shape = 0;
        if (isCircle(largest_contour_poly))
        {
            shape = 3; // Circle
        }
        else
        {
            int num_vertices = largest_contour_poly.size();
            if (num_vertices >= 3 && num_vertices <= 6)
                shape = 1; // Triangle
            else if (num_vertices > 15 && num_vertices <= 18)
                shape = 2; // Star
            else if (num_vertices >= 9 && num_vertices <= 14)
                shape = 4; // Umbrella

        }
        return shape;
    }
    return -1;
}
int main( int argc, char** argv )
{
    setup();
    cv::namedWindow("Photo");

    double Kp = 190;
    double Ki = 0;
    double Kd = 0;
    double integral = 0;
    double derivative = 0;
    double previous_error = 0;
    double setpoint = 160;
    double error = 0;

    while(1)
    {
        Mat frame;
        cv::Mat mask;

        cv::Mat hsv;
        while(frame.empty())
            frame = captureFrame();


        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);



        int shape = symbol(frame); // Detect the shape in the current frame

        switch (shape)
        {
        case 1: // triangle
            std::cout << "Detected shape: Triangle\n"<< std::endl;
// Print "red" on the console
            std::cout << "red" << std::endl;
            cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask);
            cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask);
            break;
        case 2: // Star
            std::cout << "Detected shape: Star\n"<< std::endl;
// Print "yellow" on the console
            std::cout << "yellow" << std::endl;
            cv::inRange(hsv, cv::Scalar(15, 70, 70), cv::Scalar(35, 255, 255), mask);
            break;
        case 3: // Circle
            std::cout << "Detected shape: Circle\n"<< std::endl;
// Print "blue" on the console
            std::cout << "blue" << std::endl;
            cv::inRange(hsv, cv::Scalar(100, 70, 50), cv::Scalar(130, 255, 255), mask);
            break;
        case 4: // Umbrella
            std::cout << "Detected shape: Umbrella\n"<< std::endl;
// Print "green" on the console
            std::cout << "green" << std::endl;
            cv::inRange(hsv, cv::Scalar(35, 70, 50), cv::Scalar(70, 255, 255), mask);
            break;
        default:
            std::cout << "No shape detected\n"<< std::endl;
// Print "Black" on the console
            std::cout << "Black" << std::endl;
            cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), mask);

            break;
        }
// Find contours in the mask
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// Get the first and last point of the contour and calculate the range
int firstPixel = -1;
int lastPixel = -1;
int range = -1;
 int largestContourIndex = 0;
if (!contours.empty()) {
    // Get the contour with the largest area

    double largestContourArea = cv::contourArea(contours[0]);
    for (int i = 1; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > largestContourArea) {
            largestContourIndex = i;
            largestContourArea = area;
        }
    }
    // Get the first and last point of the contour
    std::vector<cv::Point> contour = contours[largestContourIndex];
    for (int i = 0; i < contour.size(); i++) {
        int x = contour[i].x;
        if (firstPixel == -1 || x < firstPixel) {
            firstPixel = x;
        }
        if (lastPixel == -1 || x > lastPixel) {
            lastPixel = x;
        }
    }
    // Calculate the range
    range = lastPixel - firstPixel;
}

// Draw the contours on the original image (RED) and show the range on the console
if (firstPixel != -1 && lastPixel != -1) {
    cv::drawContours(frame, contours, largestContourIndex, cv::Scalar(0, 0, 255), 2);
    std::cout << "Range: " << range << ", First pixel: " << firstPixel << ", Last pixel: " << lastPixel << std::endl;
} else {
    std::cout << "No contour detected" << std::endl;
}


       double midpoint = range / 2;
        error = midpoint- setpoint;
        integral += error;
        derivative = error - previous_error;
        previous_error = error;

// Calculate the PID output
        double output = (Kp * error + Ki * integral + Kd * derivative);
// Calculate the left and right motor speeds
        int leftMotor_speed = 100; //- (int)output; Having this results in one motor being active at a time
        int rightMotor_speed = 100; //+ (int)output;


// Send the left and right motor speeds to the Arduino
        SendtoArduino(leftMotor_speed, rightMotor_speed, 86 - (int)output);

// Display the images in the windows
        cv::imshow("Photo", frame);
        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }

    closeCV(); // Disable the camera and close any windows

    return 0;
}


