#pragma once

#include <opencv2/core/utility.hpp>
#include <iostream>

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
        //! k1
        double k1;

        //! k2
        double k2;

        //! k3
        double k3;

        //! p1
        double p1;
        
        //! p2
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


        /**
         * @brief Coefficients vector.
         */
        cv::Mat vector()
        {
            return (cv::Mat_<double>(5, 1) <<
                k1, k2, p1, p2, k3);
        }
    };

    //! Distortion coefficients.
    Distortion distortion;

    //! Focal length in meters.
    double focal_length_meters;

    //! Sensor dimensions in meters.
    cv::Point2d sensor_dimensions_meters;

    //! Sensor dimensions in pixels.
    cv::Point2d sensor_dimensions_pixels;

    //! Principal point.
    cv::Point2d principal_point;

    /**
     * @brief Camera
     * Create a camera.
     */
    Camera(double _focal_length_meters, cv::Point2d _sensor_dimensions_meters,
           cv::Point2d _sensor_dimensions_pixels, cv::Point2d _principal_point,
           Distortion _distortion)
        : sensor_dimensions_meters(_sensor_dimensions_meters)
        , sensor_dimensions_pixels(_sensor_dimensions_pixels)
        , focal_length_meters(_focal_length_meters)
        , principal_point(_principal_point)
        , distortion(_distortion)
    {
    }

    /**
     * @brief Distortion coefficients vector.
     */
    cv::Mat D()
    {
        return distortion.vector();
    }

    /**
     * @brief Intrinsics matrix.
     */
    cv::Mat K()
    {
        cv::Point2d focal_length_pixels = focalLengthPixels();
        return (cv::Mat_<double>(3, 3) <<
            focal_length_pixels.x, 0, principal_point.x,
            0, focal_length_pixels.y, principal_point.y,
            0, 0, 1);
    }

    double focalLengthPixelsAspectRatio()
    /**
     * @brief focalLengthPixelsAspectRatio
     * Ratio of vertical to horizontal focal length in pixels.
     */
    {
        cv::Point2d focal_length_pixels = focalLengthPixels();
        return focal_length_pixels.y / focal_length_pixels.x;
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

    // TODO(bkd): better way to keep canned cameras?

    /**
     * @brief ParrotAnafiUSA Camera
     * Parrot Anafi USA camera information.
     */
    static Camera ParrotAnafiUSA()
    {
        double focal_length_meters = 4.04e-3;
        cv::Point2d sensor_dimensions_meters(7.22e-3, 5.50e-3);
        cv::Point2d sensor_dimensions_pixels(5344, 4016);
        cv::Point2d principal_point(sensor_dimensions_pixels.x / 2,
                                    sensor_dimensions_pixels.y / 2);
        Distortion distortion(2.8391010208309218e-02, -2.7239202041003809e-02,
                              -2.4700935014356916e-03, 6.1345950301455029e-03, 0.);

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point, distortion);
    }

    /**
     * @brief VantageVesperEONavigation Camera
     * Vantage Vesper EO Navigation camera information.
     */
    static Camera VantageVesperEONavigation() {
        double focal_length_meters = 6.00e-3;
        cv::Point2d sensor_dimensions_meters(7.68e-3, 4.32e-3);
        cv::Point2d sensor_dimensions_pixels(3840, 2160);
        cv::Point2d principal_point(sensor_dimensions_pixels.x / 2,
                                    sensor_dimensions_pixels.y / 2);

        Distortion distortion(-4.9182844322972585e-01, 2.4439190154457310e-01,
                              -1.2749662399587735e-03, 2.6276422366150747e-03, 0.);

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point, distortion);
    }
};

struct GimbalOrientation
{
    enum class Units { Degrees, Radians };

    double pitch;
    double roll;
    double yaw;
    Units units;

    GimbalOrientation(double _pitch = 0.0, double _roll = 0.0, double _yaw = 0.0, Units _units = Units::Degrees)
        : pitch(_pitch), roll(_roll), yaw(_yaw), units(_units)
    {
    }

    GimbalOrientation(const GimbalOrientation &other)
        : pitch(other.pitch), roll(other.roll), yaw(other.yaw), units(other.units)
    {
    }

    GimbalOrientation convertTo(Units _units)
    {
        if (_units == units) {
            return *this;
        }

        switch (_units) {
        case Units::Degrees:
            return GimbalOrientation(pitch * 180.0/M_PI, roll * 180.0/M_PI, yaw * 180.0/M_PI, Units::Degrees);
        case Units::Radians:
            return GimbalOrientation(pitch * M_PI/180.0, roll * M_PI/180.0, yaw * M_PI/180.0, Units::Radians);
        }
    }

    cv::Mat rotationMatrix()
    {
        GimbalOrientation gimbal_orientation = convertTo(Units::Radians);

        double x = -gimbal_orientation.pitch;
        double y = gimbal_orientation.yaw;
        double z = gimbal_orientation.roll;

        cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x)
        );

        cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
            cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y)
        );

        cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
            cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1
        );

        return (Rz * Ry) * Rx;
    }

    std::string toString(bool compact)
    {
        if (compact) {
            return "[" + std::to_string(pitch) + ", " + std::to_string(roll) + ", "
                    + std::to_string(yaw) + "]";
        } else {
            return "pitch: " + std::to_string(pitch) + " roll: "
                    + std::to_string(roll) + " yaw: " + std::to_string(yaw);
        }
    }
};

} // namespace stitcher
} // namespace airmap
