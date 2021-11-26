# crowdbot-evaluation-tools

> Repository for crowd tracking and robot performance evaluation in experiments

## File & Folder

- [`docs/`](./docs/): documentation
- [`qolo/`](./qolo/): codespace for crowdbot evaluation
- [`notebook/`](./notebook/): example notebooks for demo
  - **[intro_to_dataset.ipynb](https://github.com/epfl-lasa/crowdbot-evaluation-tools/blob/main/notebooks/intro_to_dataset.ipynb)**
  - **[comprehensive_eval.ipynb](https://github.com/epfl-lasa/crowdbot-evaluation-tools/blob/main/notebooks/comprehensive_eval.ipynb)**
- [`crowdbot_tools_archive/`](./crowdbot_tools_archive/): archive of https://github.com/danjia21/crowdbot_tools
- `dev_log.md`: update development once updated!

## Examples

<details>
    <summary><b>animation with qolo trajectories and detection/tracking results within 8 meter</b></summary> <div align="center"> <img src="./example/2021-04-10-12-36-29.gif" alt="example video"   width="500" > </div>
</details>

<details>
    <summary><b>eval_crowd demo</b></summary>
    <details>
        <summary><b>crowd_density variations along the timestamps</b></summary> <div align="center"> <img src="./example/2021-04-24-13-07-54_crowd_density.png" alt="crowd_density"   width="500" > </div>
    </details>
    <details>
        <summary><b>min_dist variations along the timestamps</b></summary> <div align="center"> <img src="./example/2021-04-24-13-07-54_min_dist.png" alt="min_dist"   width="500" > </div>
    </details>
</details>
<details open>
    <summary><b>eval_qolo demo</b></summary>
    <details>
        <summary><b>path/time to goal</b></summary> <div align="center"> <img src="./example/2021-04-24-13-07-54_path.png" alt="path"   width="500" > </div>
    </details>
    <details open>
        <summary><b>twist_acc_jerk</b></summary> <div align="center"> <img src="./example/2021-04-24-13-07-54_qolo_command.png" alt="twist_acc_jerk"   width="500" > </div>
    </details>
</details>


## Dataset

<details open>
    <summary><b>Proposed dataset structure</b></summary> <div align="center"> <img src="./example/dataset_stucture.png" alt="dataset"   width="500" > </div>
</details>

## Acknowledgment

- https://github.com/epfl-lasa/qolo-evaluation
