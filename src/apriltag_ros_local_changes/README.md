# Local changes to `apriltag_ros`

`Software/src/apriltag_ros` is vendored as a git submodule pinned to upstream commit
`71164af4e2b833a430e42996badb600fa8639a52`. These files were changed or added on top of
that commit for this project and are kept here so they aren't lost inside the pristine submodule:

- `config/tags.yaml` — our AprilTag layout: 7 standalone tags (IDs 0–6, 0.10 m) used to
  localize the robots and workspace
- `launch/continuous_detection.launch` — added args for a second (virtual) camera source
- `launch/start_rviz.launch` — new: launches RViz with our camera config
- `msg/Double_array.msg` — new: custom message type for tag detections

To reproduce the working setup, these would be copied over the corresponding files inside
`apriltag_ros/apriltag_ros/` after checking out the submodule.
