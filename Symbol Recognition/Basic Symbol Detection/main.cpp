#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
#include <thread> // for std::this_thread::sleep_for()

bool isCircle(const std::vector<cv::Point>& poly) {
    cv::RotatedRect rect = cv::fitEllipse(poly); // Fit an ellipse to the polygon
    double poly_area = cv::contourArea(poly);
    double ellipse_area = rect.size.width * rect.size.height * CV_PI / 4.0;

    // Check if the polygon is approximately circular
    return poly_area / ellipse_area >= 0.9;
}


Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void) {
  setupCamera(320, 240); // Enable the camera for OpenCV
}

int main(int argc, char **argv) {
  setup(); // Call a setup function to prepare IO and devices

  cv::namedWindow("Photo");       // Create a GUI window called photo
  cv::namedWindow("Pink Pixels"); // Create a GUI window called Pink Pixels
  cv::namedWindow("Processed Photo"); // Create a GUI window called Processed Photo

  while (1) // Main loop to perform image processing
  {
    cv::Mat frame;

    while (frame.empty()) frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

    cv::imshow("Photo", frame); // Display the image in the window

    cv::Mat hsv_frame;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV); // Convert the image to HSV color space

    // Define the lower and upper bounds of the pink color in HSV color space
    cv::Scalar lower_pink(140, 50, 50);
    cv::Scalar upper_pink(180, 255, 255);

    cv::Mat pink_mask;
    cv::inRange(hsv_frame, lower_pink, upper_pink, pink_mask); // Create a binary mask of the pink pixels

    cv::imshow("Pink Pixels", pink_mask); // Display the binary mask of the pink pixels

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(pink_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Find the contours of the binary image

    int largest_contour_index = -1;
    double largest_contour_area = 0.0;

    for (int i = 0; i < contours.size(); i++) { // Iterate over all the contours to find the largest one
      double area = cv::contourArea(contours[i]);
      if (area > largest_contour_area) {
        largest_contour_index = i;
        largest_contour_area = area;
        }
        }
        cv::Mat processed_frame = frame.clone(); // Create a clone of the original frame to draw on

        if (largest_contour_index >= 0) { // If a contour was found, draw it on the processed image
          cv::drawContours(processed_frame, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);



      std::vector<cv::Point> largest_contour_poly;
      cv::approxPolyDP(contours[largest_contour_index], largest_contour_poly, 0.01 * cv::arcLength(contours[largest_contour_index], true), true); // Approximate the largest contour to a polygon

      int num_vertices = largest_contour_poly.size();
      std::string shape;

      if (num_vertices >= 3 && num_vertices <= 6) shape = "Triangle";
      else if (num_vertices > 15 && num_vertices <= 18) shape = "Star";
      if (num_vertices >= 9 && num_vertices <= 14) {
      if (isCircle(largest_contour_poly)) {
            shape = "circle";
        } else {
            shape = "umbrella";
        }
    }

     
      std::cout << "Detected shape: " << shape << std::endl; // Print the detected shape

/*
        if (shape != "umbrella") {
          int num_vertices = largest_contour_poly.size();
          std::cout << "Number of vertices: " << num_vertices << std::endl;
        }


}
*/
cv::imshow("Processed Photo", processed_frame); // Display the processed image


cv::waitKey(1); // Wait for a key press or delay before continuing to the next iteration
}

return 0;
}
