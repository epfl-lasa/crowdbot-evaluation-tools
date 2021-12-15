# issues when exporting 0327

## Timestamp influence

> TODO: analyze one by one!

- lidar
- commands
- twist
- pose2d
- tfqolo

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

- errors

```
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
