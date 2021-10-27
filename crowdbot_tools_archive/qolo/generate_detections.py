import os
import numpy as np

from lidar_det.detector import DetectorWithClock


if __name__ == "__main__":
    ckpt_path = "/globalwork/jia/share/JRDB_cvpr21_workshop/logs/unet_bl_voxel_jrdb_0.05_0.1_20210519_232859/ckpt/ckpt_e40.pth"
    detector = DetectorWithClock(ckpt_path)

    base_dir = "/globalwork/datasets/crowdbot/qolo_market_data"
    lidar_dir = os.path.join(base_dir, "lidar")
    for seq in os.listdir(lidar_dir):
        print(seq)
        frames = os.listdir(os.path.join(lidar_dir, seq))
        frames.sort()

        det_seq_dir = os.path.join(base_dir, "detections", seq)
        os.makedirs(det_seq_dir, exist_ok=True)

        for frame in frames:
            with open(os.path.join(lidar_dir, seq, frame), "rb") as f:
                pc = np.load(f)

            boxes, scores = detector(pc.T)

            out = np.concatenate((boxes, scores[:, np.newaxis]), axis=1)
            np.savetxt(
                os.path.join(det_seq_dir, frame.replace("nby", "txt")),
                out,
                delimiter=",",
            )

        print(detector)
        detector.reset()
