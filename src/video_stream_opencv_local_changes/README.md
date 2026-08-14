# Local changes to `video_stream_opencv`

`Software/src/video_stream_opencv` is vendored as a git submodule pinned to upstream commit
`65949bdc5c9468d18c51aed9073d020bec892532`. These files were changed or added on top of
that commit for this project and are kept here so they aren't lost inside the pristine submodule:

- `src/video_stream_lego.cpp` — new: custom stream node used in place of the stock
  `video_stream` node (referenced as `video_stream_lego` from `camera.launch`)
- `config/test_calibration.yaml` — our webcam's calibration (resolution, camera/distortion matrices)
- `launch/camera.launch` — points the node type at `video_stream_lego` instead of `video_stream`
- `launch/mjpg_stream.launch` — enables the `visualize` preview window
- `launch/webcam.launch` — minor tweak
- `launch/image_process.launch`, `launch/webcamerausb.launch` — new launch files for this project's camera setup

To reproduce the working setup, these would be copied over the corresponding files inside
`video_stream_opencv/` after checking out the submodule.
