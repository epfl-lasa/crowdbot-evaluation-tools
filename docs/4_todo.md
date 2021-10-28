## TODO

- [ ] reimplement viz with open3d
  - [x] basic viz  (1026)
  - [x] display within dist (1026)
  - [x] extract robot pose in lidar frame (1027)
  - [ ] add robot pose into each frame viz
  - [ ] headless image rendering

- [ ] visualize in the world frame

  - [x] extract from bag (1026)
  - [x] store as npy (1026)

    `tf_qolo_world` `tf_qolo`

    ref: https://github.com/epfl-lasa/qolo-evaluation/blob/main/scripts/live_trajectory_plot.py

    - timestamp of tf
    - timestamp of robot

  - [ ] organize documentation

- [ ] other topic data

    - [x] visual odom -> robot trajectory (1026)
    - [x] pose 3d viz (1027)
    - [ ] robot velocity
    - [ ] synchronize all data

- [ ] refactor AllFrames class (1026)

  - [x] refactor and rename as "viz_util.py"
  - [x] image generation (1026)
  - [x] video generation (1026)
  - [x] pose extraction (1026)
  - [ ] detection
  - [ ] tracking

- [ ] robot state and control input (X and U) amd its derivative

- [ ] compact data storage, reference: other 3D detection dataset!

- [ ] visualize bbox. traj, velo in rviz (orientation)

    ref: spencer viz: https://github.com/spencer-project/spencer_people_tracking/tree/master/visualization

- [ ] consider using mutliprocessing when generating images

- [ ] consider using [ffmpeg-python](https://github.com/kkroening/ffmpeg-python) to generate videos

    ```
    import os
    os.system("ffmpeg -y -r 15 -pattern_type glob -i "tmp/*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p "tmp/frames.mp4"")
    ```

