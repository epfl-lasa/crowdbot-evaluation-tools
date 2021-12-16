# issues when exporting 0327

## Timestamp influence

- lidar
  ```shell
  lidar_stamped ['timestamp'].max()
  1616842846.7601984
  lidar_stamped ['timestamp'].min()
  1616842678.0695062
  ```

- commands:
  _commands_raw.npy: no original timestamp (extact using rosbag package)
  ```shell
  cmd_raw_dict['timestamp'].min()
  1616669108.7600613
  cmd_raw_dict['timestamp'].max()
  1616669279.065613
  ```

  _commands_sampled.npy: interpolate with lidar timestamp
- twist
  _qolo_command.npy:
  ```shell
  qolo_command_dict["timestamp"].min()
  1616669108.7600613
  qolo_command_dict["timestamp"].max()
  1616669277.4507535
  ```
- pose2d:
  _pose2d.npy: timestamp from odom topic
  ```shell
  qolo_pose2d['timestamp'].min()
  1616669108.8680496
  qolo_pose2d['timestamp'].max()
  1616669279.0651515
  ```
- tfqolo
  _qolo_state.npy:
  ```shell
  qolo_state_dict['timestamp'].min()
  1616669108.9030297
  qolo_state_dict['timestamp'].max()
  616669279.0151792
  ```
- path_eval
  ```shell
  from `pose_ts = qolo_pose2d.get("timestamp")`
  path_eval_dict['start_command_ts']
  1616669108.7600613
  path_eval_dict['end_command_ts']
  1616669225.2152271
  ```

## When running eval_qolo.py

- debug

```python
# issues when exporint 0327
# 1616842678.0695062 1616669108.7600613
# 1616842846.7601984 1616669225.2152271
# start_cmd_ts and end_cmd_ts has wrong timestamp!!!
print(cmd_ts.min(), start_cmd_ts)
print(cmd_ts.max(), end_cmd_ts)
```

- issues1

```shell
Issues found: timestamp difference because recording in wrong computer time
lidar timestamp 1616842678.0695062 1616842846.7601984
twist timestamp 1616669108.7600613 1616669279.065613

`python3 qolo/twist2npy.py --overwrite -f 0327_shared_control`
/hdd/data_qolo/crowd_qolo_recordings/0327_shared_control/2021-03-27-11-45-08.bag
/hdd/data_qolo/crowd_qolo_recordings/0327_shared_control/2021-03-27-11-48-13.bag
twist messages: 1806 / 1806
Current twist extracted!
/home/crowdbot/.local/lib/python3.8/site-packages/numpy/lib/function_base.py:1080: RuntimeWarning: invalid value encountered in true_divide
  out[tuple(slice1)] = (f[tuple(slice4)] - f[tuple(slice2)]) / (2. * ax_dx)
/home/crowdbot/.local/lib/python3.8/site-packages/numpy/lib/function_base.py:1101: RuntimeWarning: invalid value encountered in double_scalars
  out[tuple(slice1)] = (f[tuple(slice2)] - f[tuple(slice3)]) / dx_0
/home/crowdbot/.local/lib/python3.8/site-packages/numpy/lib/function_base.py:1108: RuntimeWarning: invalid value encountered in double_scalars
  out[tuple(slice1)] = (f[tuple(slice2)] - f[tuple(slice3)]) / dx_n
NaN index in x_acc: [   0    1    2 ... 3344 3345 3346]
NaN index in zrot_acc: [   0    1    2 ... 3344 3345 3346]
NaN index in x_jerk: [   0    1    2 ... 3344 3345 3346]
NaN index in zrot_jerk: [   0    1    2 ... 3344 3345 3346]
```

- issues2

```shell
(1/14): 2021-03-27-11-45-08 with 3347 frames
Traceback (most recent call last):
  File "../qolo/eval_qolo.py", line 213, in <module>
    rel_jerk = compute_rel_jerk(
  File "/home/crowdbot/Documents/yujie/lasa_crowdbot_tools/qolo/metric_qolo_perf.py", line 165, in compute_rel_jerk
    end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])
  File "<__array_function__ internals>", line 5, in argmax
  File "/home/crowdbot/.local/lib/python3.8/site-packages/numpy/core/fromnumeric.py", line 1195, in argmax
    return _wrapfunc(a, 'argmax', axis=axis, out=out)
  File "/home/crowdbot/.local/lib/python3.8/site-packages/numpy/core/fromnumeric.py", line 57, in _wrapfunc
    return bound(*args, **kwds)
ValueError: attempt to get argmax of an empty sequenc
```

- fix issues by computing adding code in twist2npy.py
  ```python
  if min(interp_ts) > max(source_ts):
    # existing 0327 data has wrong timestamps
    print("Warning: all interp_ts are larger than source_ts")
    interp_ts -= min(interp_ts) - min(source_ts)
  ```
