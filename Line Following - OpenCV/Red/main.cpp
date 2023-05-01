// Include files for required libraries
#include <stdio.h>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
#include <thread> // for std::this_thread::sleep_for()

Pi2c car(0x04); // Configure the I2C interface to the Car as a global variable

void setup(void)
{
    setupCamera(320, 240); // Enable the camera for OpenCV
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

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo");   // Create a GUI window called photo
    // Initialize variables for PID control
    double Kp = 190;    // Proportional gain
    double Ki = 0;   // Integral gain
    double Kd = 0;   // Derivative gain
    double integral = 0;
    double derivative = 0;
    double previous_error = 0;
    double setpoint = 160;   // Desired position of the car
    double error = 0;

    while(1)    // Main loop to perform image processing
    {

        Mat frame;

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        // Convert the image to HSV color space
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);



        // Threshold the image to extract the red pixels
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);
        cv::Mat mask = mask1 | mask2;

        // Apply a morphological opening to remove noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        // Apply erosion and dilation to the mask
        cv::erode(mask, mask, kernel); // Erode the mask with the structuring element
        cv::dilate(mask, mask, kernel); // Dilate the mask with the structuring element

        // Convert the non-red pixels to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        gray.setTo(0, mask);

        // Apply Gaussian blurring to reduce noise
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Apply a threshold to create a binary image
        Mat binary;
        threshold(gray, binary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

        // Create a mask to remove any white pixels that are part of the background
        Mat bgMask;
        threshold(gray, bgMask, 240, 255, THRESH_BINARY);

        // Invert the mask to make white pixels 0 and black pixels 255
        bgMask = 255 - bgMask;

        // Apply the mask to the binary image
        bitwise_and(binary, bgMask, binary);

        // Create a mask to remove any black pixels that are not part of the tape
        Mat tapeMask;
        morphologyEx(binary, tapeMask, MORPH_OPEN, kernel);

        // Find contours in the mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw the contours on the original image ( RED)
         cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);


        // Get the colors of the bottom row of pixels
        int y = frame.rows - 50;
        std::vector<cv::Vec3b> colors;
        for (int x = 0; x < frame.cols; x++) {
            cv::Vec3b color = frame.at<cv::Vec3b>(y, x);
            colors.push_back(color);
        }

        // Find the bottom row of the binary image
        cv::Mat bottomRow = mask.row(mask.rows-50);


        // Find the range of red pixels
        int start = -1, end = -1;
        for (int i = 0; i < colors.size(); i++) {
            cv::Vec3b color = colors[i];
            if (color[2] > 150 && color[1] < 100 && color[0] < 100) {
                if (start == -1) {
                    start = i;
                }
                end = i;
            }
        }


      
        // Print the range of pixels to the console
        if (start != -1) {
            printf("The range of pixels is %d-%d\n", start, end);
            int midpoint = (start + end) / 2;
            error = midpoint - setpoint;
            integral += error;
            derivative = error - previous_error;
            previous_error = error;
        }

        else {
            printf("No pixels found\n");
        }

        // Calculate the PID output
        double output = (Kp * error + Ki * integral + Kd * derivative);
        // Calculate the left and right motor speeds
        int leftMotor_speed = 100; //- (int)output; Having this results in one motor being active at a time
        int rightMotor_speed = 100; //+ (int)output;

        // Limit the output to the range [-40, 70]
        if (output > 70)
        {
            output = 70;
        }
        else if (output < -40)
        {
            output = -40;
        }

        // Limit the motor speeds to the range [0, 150]
        if (leftMotor_speed > 150)
        {
            leftMotor_speed = 150;
        }
        else if (leftMotor_speed < 0)
        {
            leftMotor_speed = 0;
        }
        if (rightMotor_speed > 150)
        {
            rightMotor_speed = 150;
        }
        else if (rightMotor_speed < 0)
        {
            rightMotor_speed = 0;
        }


        // Send the left and right motor speeds to the Arduino
        SendtoArduino(leftMotor_speed, rightMotor_speed, 86 - (int)output);

        // Display the images in the windows
        cv::imshow("Photo", frame);
        cv::imshow("Gray", gray);
        cv::imshow("binary", tapeMask);

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }

    closeCV();  // Disable the camera and close any windows

    return 0;
}
