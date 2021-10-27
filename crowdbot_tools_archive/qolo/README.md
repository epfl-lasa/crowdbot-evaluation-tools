Make a `data` directory with the following layout
```
crowdbot_tools
├── data
│   ├── qolo_market_data
│   │   ├── rosbag
│   │   ├── lidar
│   │   ├── detections
│   │   ├── tracks
...
``` 
Download the rosbag and put them under `rosbag`.
Then run
`python2 extract_lidar_from_rosbag.py` to populate `lidar` directory.
Download and put the pre-generated detections and tracks in the respective folders.
See `run.sh` for examples on running the provided scripts.