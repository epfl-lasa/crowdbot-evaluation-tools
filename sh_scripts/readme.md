# sh_scripts/

## Basic uses

- Parameters
    - `-e`: conda environment
    - `-t`: target folder to evaluate from the crowdbot dataset
- Example

```shell
bash data_export_eval_all.sh -e=py38cuda110 -t=1203_test

chmod +x data_export_eval_all.sh
./data_export_eval_all.sh -e=py38cuda110 -t=1203_test
```

## Different scripts

| Scripts/Included functions | extract_lidar | source data to npy | algorithm | eval_qolo_path | eval_qolo | eval_crowd |
| -------------------------- | ------------- | ------------------ | --------- | -------------- | --------- | ---------- |
| `eval_all.sh`              |               |                    |           | ✔️              | ✔️         | ✔️          |
| `export_all.sh`            | ✔️             | ✔️                  |           |                |           |            |
| `export_eval_qolo.sh`      | ✔️             | ✔️                  |           | ✔️              | ✔️         |            |
| `export_eval_all.sh`       | ✔️             | ✔️                  | ✔️         | ✔️              | ✔️         | ✔️          |

PS: for `eval_all.sh` , lidar and corresponding time stamps is required, and this script aims to **update extracted source data and evaluation results**.
