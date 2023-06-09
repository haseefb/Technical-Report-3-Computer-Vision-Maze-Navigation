#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    // Read the image
    Mat img = imread("img.bmp");

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
    Scalar lower_orange = Scalar(11, 70, 50);
    Scalar upper_orange = Scalar(20, 255, 255);
    Scalar lower_yellow = Scalar(21, 70, 50);
    Scalar upper_yellow = Scalar(30, 255, 255);
    Scalar lower_green = Scalar(31, 70, 50);
    Scalar upper_green = Scalar(80, 255, 255);
    Scalar lower_turquoise = Scalar(81, 70, 50);
    Scalar upper_turquoise = Scalar(100, 255, 255);
    Scalar lower_blue = Scalar(101, 70, 50);
    Scalar upper_blue = Scalar(130, 255, 255);
    Scalar lower_purple = Scalar(131, 70, 50);
    Scalar upper_purple = Scalar(145, 255, 255);
    Scalar lower_pink = Scalar(146, 70, 50);
    Scalar upper_pink = Scalar(170, 255, 255);
    Scalar lower_black = Scalar(0, 0, 0);
    Scalar upper_black = Scalar(179, 255, 30);
    Scalar lower_white = Scalar(0, 0, 231);
    Scalar upper_white = Scalar(179, 30, 255);

    // Create a mask for each color
    Mat red_mask, orange_mask, yellow_mask, green_mask, turquoise_mask, blue_mask, purple_mask, pink_mask, black_mask, white_mask;
    inRange(hsv, lower_red, upper_red, red_mask);
    inRange(hsv, lower_orange, upper_orange, orange_mask);
    inRange(hsv, lower_yellow, upper_yellow, yellow_mask);
    inRange(hsv, lower_green, upper_green, green_mask);
    inRange(hsv, lower_turquoise, upper_turquoise, turquoise_mask);
    inRange(hsv, lower_blue, upper_blue, blue_mask);
    inRange(hsv, lower_purple, upper_purple, purple_mask);
    inRange(hsv, lower_pink, upper_pink, pink_mask);
    inRange(hsv, lower_black, upper_black, black_mask);
    inRange(hsv, lower_white, upper_white, white_mask);

  // Count the number of pixels in each color mask
int red_pixels = countNonZero(red_mask);
int blue_pixels = countNonZero(blue_mask);
int green_pixels = countNonZero(green_mask);
int yellow_pixels = countNonZero(yellow_mask);
int orange_pixels = countNonZero(orange_mask);
int purple_pixels = countNonZero(purple_mask);
int turquoise_pixels = countNonZero(turquoise_mask);
int pink_pixels = countNonZero(pink_mask);


// Determine the dominant color
Mat dominant_img;
string message;
if (red_pixels > blue_pixels && red_pixels > green_pixels && red_pixels > yellow_pixels && red_pixels > orange_pixels && red_pixels > purple_pixels && red_pixels > turquoise_pixels && red_pixels > pink_pixels) {
    // Highlight the red color in the masked image
    Mat red_highlight;
    masked_img.copyTo(red_highlight, red_mask);
    dominant_img = red_highlight;
    message = "Dominant Color: Red";
}
else if (blue_pixels > red_pixels && blue_pixels > green_pixels && blue_pixels > yellow_pixels && blue_pixels > orange_pixels && blue_pixels > purple_pixels && blue_pixels > turquoise_pixels && blue_pixels > pink_pixels) {
    // Highlight the blue color in the masked image
    Mat blue_highlight;
    masked_img.copyTo(blue_highlight, blue_mask);
    dominant_img = blue_highlight;
    message = "Dominant Color: Blue";
}
else if (green_pixels > red_pixels && green_pixels > blue_pixels && green_pixels > yellow_pixels && green_pixels > orange_pixels && green_pixels > purple_pixels && green_pixels > turquoise_pixels && green_pixels > pink_pixels) {
// Highlight the green color in the masked image
Mat green_highlight;
masked_img.copyTo(green_highlight, green_mask);
dominant_img = green_highlight;
message = "Dominant Color: Green";
}
else if (yellow_pixels > red_pixels && yellow_pixels > blue_pixels && yellow_pixels > green_pixels && yellow_pixels > orange_pixels && yellow_pixels > purple_pixels && yellow_pixels > turquoise_pixels && yellow_pixels > pink_pixels) {
// Highlight the yellow color in the masked image
Mat yellow_highlight;
masked_img.copyTo(yellow_highlight, yellow_mask);
dominant_img = yellow_highlight;
message = "Dominant Color: Yellow";
}
else if (orange_pixels > red_pixels && orange_pixels > blue_pixels && orange_pixels > green_pixels && orange_pixels > yellow_pixels && orange_pixels > purple_pixels && orange_pixels > turquoise_pixels && orange_pixels > pink_pixels) {
// Highlight the orange color in the masked image
Mat orange_highlight;
masked_img.copyTo(orange_highlight, orange_mask);
dominant_img = orange_highlight;
message = "Dominant Color: Orange";
}
else if (purple_pixels > red_pixels && purple_pixels > blue_pixels && purple_pixels > green_pixels && purple_pixels > yellow_pixels && purple_pixels > orange_pixels && purple_pixels > turquoise_pixels && purple_pixels > pink_pixels) {
// Highlight the purple color in the masked image
Mat purple_highlight;
masked_img.copyTo(purple_highlight, purple_mask);
dominant_img = purple_highlight;
message = "Dominant Color: Purple";
}
else if (turquoise_pixels > red_pixels && turquoise_pixels > blue_pixels && turquoise_pixels > green_pixels && turquoise_pixels > yellow_pixels && turquoise_pixels > orange_pixels && turquoise_pixels > purple_pixels && turquoise_pixels > pink_pixels) {
// Highlight the turquoise color in the masked image
Mat turquoise_highlight;
masked_img.copyTo(turquoise_highlight, turquoise_mask);
dominant_img = turquoise_highlight;
message = "Dominant Color: Turquoise";
}
else if (pink_pixels > red_pixels && pink_pixels > blue_pixels && pink_pixels > green_pixels && pink_pixels > yellow_pixels && pink_pixels > orange_pixels && pink_pixels > purple_pixels && pink_pixels > turquoise_pixels) {
// Highlight the pink color in the masked image
Mat pink_highlight;
masked_img.copyTo(pink_highlight, pink_mask);
dominant_img = pink_highlight;
message = "Dominant Color: Pink";
}
else {
// No dominant color was found
dominant_img = img;
message = "No Dominant Color Found";
}
 imshow("Original Image", img);

    // Display the masked image with the dominant color highlighted
    imshow(message, dominant_img);

    // Display the message
    cout << message << endl;

    waitKey(0);

    return 0;
}
