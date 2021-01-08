#pragma once

#include <boost/optional.hpp>
#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {

/**
 * @brief Camera information
 * Contains information about the camera, including
 * intrinsics, distortion coefficients, and field of view.
 */
struct Camera
{
    enum class FOVUnits { Degrees, Radians };

    /**
     * @brief CameraDistortion
     * Camera distortion coefficients.
     */
    struct Distortion
    {
        double k1;
        double k2;
        double k3;
        double p1;
        double p2;

        /**
         * @brief Distortion.
         * Create camera distortion coefficients.
         * The order of parameters is consistent with OpenCV.
         */
        Distortion(double _k1, double _k2, double _p1, double _p2, double _k3)
            : k1(_k1), k2(_k2), k3(_k3), p1(_p1), p2(_p2)
        {
        }

        Distortion()
            : k1(0.0), k2(0.0), k3(0.0), p1(0.0), p2(0.0)
        {
        }

        bool operator==(const Distortion &other) const
        {
            return k1 == other.k1 &&
                   k2 == other.k2 &&
                   k3 == other.k3 &&
                   p1 == other.p1 &&
                   p2 == other.p2;
        }

        /**
         * @brief Coefficients vector.
         */
        cv::Mat vector()
        {
            return (cv::Mat_<double>(5, 1) <<
                k1, k2, p1, p2, k3);
        }

        bool vectorEquals(cv::Mat &other)
        {
            cv::Mat vector_ = vector();
            return vector_.at<double>(0, 0) == other.at<double>(0, 0) &&
                   vector_.at<double>(1, 0) == other.at<double>(1, 0) &&
                   vector_.at<double>(2, 0) == other.at<double>(2, 0) &&
                   vector_.at<double>(3, 0) == other.at<double>(3, 0) &&
                   vector_.at<double>(4, 0) == other.at<double>(4, 0);
        }
    };

    /**
     * @brief calibration_intrinsics;
     * Intrinsic matrix from calibration.
     */
    boost::optional<cv::Mat> calibration_intrinsics;

    /**
     * @brief distortion
     * Distortion coefficients.
     */
    Distortion distortion;

    /**
     * @brief focal_length_meters
     * Focal length in meters.
     */
    double focal_length_meters;

    /**
     * @brief sensor_dimensions_meters
     * Sensor dimensions in meters.
     */
    cv::Point2d sensor_dimensions_meters;

    /**
     * @brief sensor_dimensions_pixels
     * Sensor dimensions in pixels.
     */
    cv::Point2d sensor_dimensions_pixels;

    /**
     * @brief principal_point
     * Principal point.
     */
    cv::Point2d principal_point;

    /**
     * @brief Camera
     * Create a camera.
     */
    Camera(double _focal_length_meters, cv::Point2d _sensor_dimensions_meters,
           cv::Point2d _sensor_dimensions_pixels, cv::Point2d _principal_point,
           Distortion _distortion,
           boost::optional<cv::Mat> _calibration_intrinsics = boost::optional<cv::Mat>())
        : focal_length_meters(_focal_length_meters)
        , sensor_dimensions_meters(_sensor_dimensions_meters)
        , sensor_dimensions_pixels(_sensor_dimensions_pixels)
        , principal_point(_principal_point)
        , distortion(_distortion)
        , calibration_intrinsics(_calibration_intrinsics)
    {
    }

    Camera()
        : focal_length_meters(0)
        , sensor_dimensions_meters(cv::Point2d(0, 0))
        , sensor_dimensions_pixels(cv::Point2d(0, 0))
        , principal_point(cv::Point2d(0, 0))
    {
    }

    /**
     * @brief focalLengthPixels
     * Focal length in pixels.
     */
    cv::Point2d focalLengthPixels()
    {
        double x = focal_length_meters * sensor_dimensions_pixels.x / sensor_dimensions_meters.x;
        double y = focal_length_meters * sensor_dimensions_pixels.y / sensor_dimensions_meters.y;
        return cv::Point2d(x, y);
    }

    /**
     * @brief fov
     * Horizontal and vertical fields of view.
     * @params units - degrees or radians
     */
    cv::Point2d fov(FOVUnits units = FOVUnits::Radians)
    {
        double x = 2 * atan(sensor_dimensions_meters.x / (2 * focal_length_meters));
        double y = 2 * atan(sensor_dimensions_meters.y / (2 * focal_length_meters));

        if (units == FOVUnits::Degrees) {
            x *= 180 / M_PI;
            y *= 180 / M_PI;
        }

        return cv::Point2d(x, y);
    }

    /**
     * @brief Intrinsics matrix.
     * @param scale
     */
    cv::Mat K(double scale = 1.0)
    {
        if (calibration_intrinsics.has_value()) {
            cv::Mat K;
            calibration_intrinsics.value().copyTo(K);
            K.at<double>(0, 0) *= scale;
            K.at<double>(0, 2) *= scale;
            K.at<double>(1, 1) *= scale;
            K.at<double>(1, 2) *= scale;
            return K;
        }

        cv::Point2d focal_length_pixels = focalLengthPixels();
        return (cv::Mat_<double>(3, 3) <<
                focal_length_pixels.x * scale, 0, principal_point.x * scale,
                0, focal_length_pixels.y * scale, principal_point.y * scale,
                0, 0, 1);
    }
};

} // namespace stitcher
} // namespace airmap
