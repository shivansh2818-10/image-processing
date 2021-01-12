#pragma once

#include "camera.h"
#include "panorama.h"

#include <boost/optional.hpp>
#include <iostream>

namespace airmap {
namespace stitcher {

/**
 * @brief CameraModels
 * Canned camera modelss.
 */
struct CameraModels
{
    std::map<std::string, Camera> models;

    CameraModels()
    {
        models["AnafiThermal"] = ParrotAnafiThermal();
        models["GreenSeer EO Navigation Lens"] = VantageVesperEONavigation();
    }

    /**
     * @brief detect
     * Detect the camera model and return a Camera instance.
     * @param image
     */
    boost::optional<Camera> detect(const GeoImage &image)
    {
        for (const auto &model : models) {
            if (image.cameraModel == model.first) {
                return model.second;
            }
        }

        return boost::optional<Camera>();
    }

    /**
     * @brief ParrotAnafiThermal Camera
     * Parrot Anafi Thermal camera information.
     */
    static Camera ParrotAnafiThermal()
    {
        double focal_length_meters = 4.04e-3;
        cv::Point2d sensor_dimensions_meters(7.22e-3, 5.50e-3);
        cv::Point2d sensor_dimensions_pixels(5344, 4016);
        cv::Point2d principal_point(sensor_dimensions_pixels.x / 2,
                                    sensor_dimensions_pixels.y / 2);
        Camera::Distortion distortion(2.8391010208309218e-02, -2.7239202041003809e-02,
                              -2.4700935014356916e-03, 6.1345950301455029e-03, 0.0);
        cv::Mat calibration_intrinsics = (cv::Mat_<double>(3, 3) <<
            2.9803291905661031e+03, 0.0, 2.3574166587045511e+03,
            0.0, 2.9704815213723482e+03, 1.6709805782521339e+03,
            0.0, 0.0, 1.0);

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point, distortion,
                      calibration_intrinsics);
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
        Camera::Distortion distortion(-4.9182844322972585e-01, 2.4439190154457310e-01,
                              -1.2749662399587735e-03, 2.6276422366150747e-03, 0.0);
        cv::Mat calibration_intrinsics = (cv::Mat_<double>(3, 3) <<
            1.5274639601248955e+03, 0.0, 9.4516465431933693e+02,
            0.0, 1.5265816612763106e+03, 6.0715982114869405e+02,
            0.0, 0.0, 1.0);

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point, distortion);
    }
};

} // namespace stitcher
} // namespace airmap
