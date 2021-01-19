#include "gtest/gtest.h"
#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/distortion.h"
#include "airmap/panorama.h"

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using airmap::stitcher::CameraModels;
using airmap::stitcher::Camera;
using airmap::stitcher::GeoImage;
using boost::filesystem::path;

TEST(cameraModels, models)
{
    CameraModels models = CameraModels();
    Camera anafi_thermal = CameraModels::ParrotAnafiThermal();
    Camera vesper = CameraModels::VantageVesperEONavigation();

    ASSERT_DOUBLE_EQ(anafi_thermal.focal_length_meters, models.models["AnafiThermal"].focal_length_meters);
    ASSERT_DOUBLE_EQ(vesper.focal_length_meters, models.models["GreenSeer EO Navigation Lens"].focal_length_meters);
}

TEST(cameraModels, detect)
{
    path image_directory = (path("../") / path("test") / path("fixtures") /
                            path("panorama_aus_1"));
    GeoImage image = GeoImage::fromExif((image_directory / path("P5050970.JPG")).string());
    Camera expected = CameraModels::ParrotAnafiThermal();
    CameraModels models;
    boost::optional<Camera> actual = models.detect(image);

    ASSERT_TRUE(actual.has_value());
    ASSERT_DOUBLE_EQ(actual.get().focal_length_meters, expected.focal_length_meters);
}
