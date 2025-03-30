// #include <ros/node_handle.h>
// #include <ros/publisher.h>
// #include <ros/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <memory>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/* Color Space Conversion:
*RGB to HSV : Working in the HSV(Hue, Saturation, Value) color space is
generally preferred over RGB for color-based object detection.
     * Hue represents the color itself (e.g., red, green, blue).
     * Saturation represents the color's purity.
     * Value represents the color's brightness.
     * HSV allows for more robust color segmentation, as it's less sensitive to
lighting changes.
     * In OpenCV, you can convert from RGB to HSV using cv::cvtColor(image,
hsv_image, cv::COLOR_BGR2HSV);.

 * Color Masking and Filtering:
   * General Color Range: Instead of creating a mask for a single specific
color, you can define broader ranges for hue, saturation, and value. For
example, to detect "any" color, you might need to test several hue ranges.
   * Creating Masks:
     * Use cv::inRange(hsv_image, lower_bound, upper_bound, mask);
     to create a binary mask.
     * lower_bound and upper_bound are cv::Scalar values representing the
minimum and maximum HSV values you want to include.
     * You can create multiple masks for different color ranges and combine them
using bitwise OR (cv::bitwise_or).
   * Noise Reduction:
     * Apply morphological operations (e.g., erosion, dilation, opening,
closing) to remove noise and smooth the mask.
*cv::erode(), cv::dilate(), cv::morphologyEx().
*Gaussian blur cv::GaussianBlur() can also help.

*Contour Detection :
*Find Contours
    : Use cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL,
                           cv::CHAIN_APPROX_SIMPLE);
to find the contours in the masked image.

   * Filter Contours:
     * Filter contours based on their area, perimeter, and shape to identify
potential balls.
     * cv::contourArea() to get the area.
     * cv::arcLength() to get the perimeter.
     * cv::approxPolyDP() to approximate the contour with a polygon.
* Circle Detection:
       * cv::minEnclosingCircle() is especially helpful for finding the center
and radius of the detected circular objects.
 * Ball Detection and Tracking:
   * Circle Fitting: Use cv::minEnclosingCircle() to find the minimum enclosing
circle for each contour.
   * Filtering Circles: Filter circles based on their area, roundness, and
aspect ratio to ensure they are likely to be balls.
   * Tracking: If you need to track the balls over time, you can use tracking

algorithms (e.g., Kalman filter, meanshift, or correlation trackers) to maintain
their identities.
* ROS Integration:
   * Image Subscriptions: Subscribe to the camera's image topic in your ROS
node.
   * OpenCV Processing: Apply the OpenCV processing steps described above to the
received image.
   * Publishing Results: Publish the detected ball positions (e.g., center
coordinates, radius) as ROS messages.
*/

class ImageProcessing {
public:
  ImageProcessing() {
    this->ImageSub = n.subscribe("/camera/rgb/image_raw", 10,
                                 &ImageProcessing::receiveImageCallback, this);
    this->ObjectPosePub =
        n.advertise<geometry_msgs::Point>("/rover/object_coordinates", 10);
  }
  /**
   * This Function is the callback function to be called every time a new Image
   * is published. The Function Will be reesponsible for process the Image and
   * detect the object and extract its position
   * Steps:
   *     -> Convert The Image (Ros Image) to CV image using cv_bridge
   *     -> convert the CV Image to HSV Image
   *     -> get a binary mask based on a lower bound and upper bound values
   *     -> upper bound and lower bound are of data type cv::Scaler types
   *     -> apply cv::erode(), cv::dilate() to enahnce the mask countors
   */
  void receiveImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
    cv_bridge::CvImagePtr cvPtr;
    try {
      cvPtr =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cvPtr->image;
    cv::Mat Img_hsv, binaryMask;

    // Convert from RGB to HSV colorspace
    cv::cvtColor(image, Img_hsv, cv::COLOR_BGR2HSV);

    // Detect the object based on HSV Range Values
    cv::Scalar lowerbound(hsv_color_[0] - tolerance_,
                          hsv_color_[1] - tolerance_,
                          hsv_color_[2] - tolerance_);
    cv::Scalar upperbound(hsv_color_[0] + tolerance_,
                          hsv_color_[1] + tolerance_,
                          hsv_color_[2] + tolerance_);

    // Ensure HSV values are within valid ranges
    lowerbound[0] = std::max(0.0, std::min(179.0, lowerbound[0]));
    lowerbound[1] = std::max(0.0, std::min(255.0, lowerbound[1]));
    lowerbound[2] = std::max(0.0, std::min(255.0, lowerbound[2]));
    upperbound[0] = std::max(0.0, std::min(179.0, upperbound[0]));
    upperbound[1] = std::max(0.0, std::min(255.0, upperbound[1]));
    upperbound[2] = std::max(0.0, std::min(255.0, upperbound[2]));

    // Get the Mask based on the lower and upper bound (all the pixels in the
    // range will be 1 others eill be 0)
    cv::inRange(Img_hsv, lowerbound, upperbound, binaryMask);

    cv::erode(binaryMask, binaryMask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(binaryMask, binaryMask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(binaryMask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours) {
      double area = cv::contourArea(contour);
      if (area > 100) { // Filter small contours
        cv::Point2f center;
        float radius;

        cv::minEnclosingCircle(contour, center, radius);

        if (radius > 10) { // Filter small circles

          cv::circle(image, center, static_cast<int>(radius),
                     cv::Scalar(0, 255, 0), 2);

          geometry_msgs::Point ball_point;
          ball_point.x = center.x;
          ball_point.y = center.y;
          ball_point.z = radius;
          ObjectPosePub.publish(ball_point);
        }
      }
    }
  }
  static void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
      std::shared_ptr<ImageProcessing> detector(
          static_cast<ImageProcessing *>(userdata));
      cv::Mat rgb_pixel(1, 1, CV_8UC3, detector->image_.at<cv::Vec3b>(y, x));
      cv::Mat hsv_pixel_mat;
      cv::cvtColor(rgb_pixel, hsv_pixel_mat, cv::COLOR_BGR2HSV);
      detector->hsv_color_[0] = hsv_pixel_mat.at<cv::Vec3b>(0, 0)[0];
      detector->hsv_color_[1] = hsv_pixel_mat.at<cv::Vec3b>(0, 0)[1];
      detector->hsv_color_[2] = hsv_pixel_mat.at<cv::Vec3b>(0, 0)[2];

      detector->color_selected_ = true;
      ROS_INFO("Color selected: H=%d, S=%d, V=%d", detector->hsv_color_[0],
               detector->hsv_color_[1], detector->hsv_color_[2]);
    }
  }

private:
  ros::NodeHandle n;
  ros::Subscriber ImageSub;
  ros::Subscriber LidarPointSub;
  ros::Publisher ObjectPosePub;
  cv::Vec3i hsv_color_;
  bool color_selected_ = false;
  cv::Mat image_;
  int tolerance_{20};
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "image_processing");
  std::cout << CV_MAJOR_VERSION << "\n";
  ImageProcessing imageProcessing;
  ros::spin();
  return 0;
}
