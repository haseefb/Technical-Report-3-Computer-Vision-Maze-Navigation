#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    // Read the image
    Mat img = imread("RedCar.bmp");

    // Convert the image to grayscale
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // Threshold the image to create a mask
    Mat mask;
    threshold(gray, mask, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    // Apply the mask to the image to remove the background
    Mat masked_img;
    img.copyTo(masked_img, mask);

    // Convert the masked image to HSV color space
    Mat hsv;
    cvtColor(masked_img, hsv, COLOR_BGR2HSV);

    // Define the range of colors you want to detect
    Scalar lower_red = Scalar(0, 70, 50);
    Scalar upper_red = Scalar(10, 255, 255);
    Scalar lower_blue = Scalar(100, 50, 50);
    Scalar upper_blue = Scalar(130, 255, 255);
    Scalar lower_green = Scalar(40, 50, 50);
    Scalar upper_green = Scalar(70, 255, 255);

    // Create a mask for each color
    Mat red_mask, blue_mask, green_mask;
    inRange(hsv, lower_red, upper_red, red_mask);
    inRange(hsv, lower_blue, upper_blue, blue_mask);
    inRange(hsv, lower_green, upper_green, green_mask);

    // Count the number of pixels in each color mask
    int red_pixels = countNonZero(red_mask);
    int blue_pixels = countNonZero(blue_mask);
    int green_pixels = countNonZero(green_mask);

    // Determine the dominant color
    Mat dominant_img;
    string message;
    if (red_pixels > blue_pixels && red_pixels > green_pixels) {
        // Highlight the red color in the masked image
        Mat red_highlight;
        masked_img.copyTo(red_highlight, red_mask);
        dominant_img = red_highlight;
        message = "Dominant Color: Red";
    }
    else if (blue_pixels > red_pixels && blue_pixels > green_pixels) {
        // Highlight the blue color in the masked image
        Mat blue_highlight;
        masked_img.copyTo(blue_highlight, blue_mask);
        dominant_img = blue_highlight;
        message = "Dominant Color: Blue";
    }
    else if (green_pixels > red_pixels && green_pixels > blue_pixels) {
        // Highlight the green color in the masked image
        Mat green_highlight;
        masked_img.copyTo(green_highlight, green_mask);
        dominant_img = green_highlight;
        message = "Dominant Color: Green";
    }
    else {
        // No dominant color was found
        dominant_img = img;
        message = "No Dominant Color Found";
    }

    // Display the original image
    imshow("Original Image", img);
    imshow("Grayscale", mask);
    imshow("Masked Image", masked_img);
    imshow("HSV Image", hsv);
    imshow("Red Image", red_mask);
    imshow("Blue Image", blue_mask);
    imshow("Green Image", green_mask);

    // Display the masked image with the dominant color highlighted
    imshow(message, dominant_img);

    // Display the message
    cout << message << endl;
    cout << "Red Pixels: " << red_pixels << endl;
    cout << "Green Pixels: " << green_pixels << endl;
    cout << "Blue Pixels: " << blue_pixels << endl;
    waitKey(0);
    return 0;
}
