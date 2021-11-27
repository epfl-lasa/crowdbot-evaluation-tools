# sh_scripts/

## Basic uses

```
sh data_eval_all.sh -e=py38cuda110 -t=shared_test
```

### Parameters

- `-e`: conda environment
- `-t`: target test folder in the crowdbot database

## Different scripts

| Scripts/Included functions        | extract_lidar | tfqolo2npy | twist2npy | pose2d2npy | algorithm | eval_qolo | eval_crowd |
| --------------------------------- | ------------- | ---------- | --------- | ---------- | --------- | --------- | ---------- |
| `data_eval_all.sh`                |               |            |           |            |           | ✔️         | ✔️          |
| `data_export_eval_qolo.sh`        | ✔️             | ✔️          | ✔️         | ✔️          |           | ✔️         |            |
| `data_export_eval_source_data.sh` |               | ✔️          | ✔️         | ✔️          |           | ✔️         | ✔️          |
| `data_export_eval_all.sh`         | ✔️             | ✔️          | ✔️         | ✔️          | ✔️         | ✔️         | ✔️          |

