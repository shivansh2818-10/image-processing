#!/usr/bin/env bats

@test "stitch basic panorama" {
  input_images=$(find ../test/fixtures/panorama_aus_1 -name *.JPG | xargs)
  output=$(./airmap_stitcher $input_images)
  [ "$output" = "[stitcher]>Written stitched image to ./panorama.jpg" ]
  panorama_size=$(stat -c %s panorama.jpg | numfmt --to=iec)
  [ "$panorama_size" = "3.0M" ]
}
