# qolo/ros

## Modules

### Preprocess

> Export each video (with bouding boxes) as mp4 --> then perform a deface --> then save it again as a rosbag (bag_filter_image.py)

## TODO

- [x] extract the rgbd data maybe as a video (data files)
    - ref: bag to rgb and depth: https://github.com/nihalsoans91/Bag_to_Depth
- [ ] filter the extracted image or video with deface
    - ref: deface link: https://github.com/ORB-HD/deface
- [ ] write back defaced rgb image back to rosbag
    - ref: create rosbag back into rosbag
        1. https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_bagcreater
        2. https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/image_folder_to_rosbag.py