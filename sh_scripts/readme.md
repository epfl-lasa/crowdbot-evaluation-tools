# sh_scripts/

## Basic uses

```
sh data_eval_all.sh -e=py38cuda110 -t=shared_test
```

### Parameters

- `-e`: conda environment
- `-t`: target test folder in the crowdbot database

## Different scripts

| Scripts/Included functions        | extract_lidar | source data to npy | algorithm | eval_qolo | eval_crowd |
| --------------------------------- | ------------- | ------------------ | --------- | --------- | ---------- |
| `eval_all.sh`                     |               |                    |           | ✔️         | ✔️          |
| `data_export_eval_source_data.sh` |               | ✔️                  |           | ✔️         | ✔️          |
| `data_export_eval_qolo.sh`        | ✔️             | ✔️                  |           | ✔️         |            |
| `data_export_eval_all.sh`         | ✔️             | ✔️                  | ✔️         | ✔️         | ✔️          |

PS: for `data_eval_all.sh` and `data_eval_all.sh`, lidar and corresponding time stamps is required. These two scripts aim to **update extracted source data and evaluation results**.

