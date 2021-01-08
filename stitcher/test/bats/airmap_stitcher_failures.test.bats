@test "too few images" {
  run ./airmap_stitcher ../test/fixtures/panorama_aus_1/P5050970.JPG
  [ "$status" -eq 1 ]
  [ "$output" = "Can't stitch; need more images" ]
}
