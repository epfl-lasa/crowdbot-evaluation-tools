import os
import numpy as np

from common import AllFrames, plot_frame
from AB3DMOT.AB3DMOT_libs.model import AB3DMOT


def reorder(boxes):
    # from x, y, z,  l,  w, h, theta (lidar frame: x-forward, y-left, z-up)
    # to   h, w, l, -y, -z, x, theta (cam frame: z-forward, x-right, y-down)
    inds = [5, 4, 3, 1, 2, 0, 6]
    boxes = boxes[:, inds]
    boxes[:, 3] *= -1
    boxes[:, 4] *= -1
    return boxes


def reorder_back(boxes):
    # from h, w, l, -y, -z, x, theta, ID
    # to   x, y, z,  l,  w, h, theta, ID
    inds = [5, 3, 4, 2, 1, 0, 6, 7]
    boxes = boxes[:, inds]
    boxes[:, 1] *= -1
    boxes[:, 2] *= -1
    return boxes


if __name__ == "__main__":
    min_conf = 0.5
    allf = AllFrames()
    for seq_idx in range(allf.nr_seqs()):
        tracker = AB3DMOT(max_age=2, min_hits=3)

        for fr_idx in range(allf.nr_frames(seq_idx)):
            lidar, dets, dets_conf, _ = allf[seq_idx, fr_idx]

            dets = dets[dets_conf > min_conf]
            dets = reorder(dets)
            trk_input = {"dets": dets, "info": np.zeros_like(dets)}
            trks = tracker.update(trk_input)
            trks = reorder_back(trks)

            # plot_frame(lidar, trks, out_path=f"./tmp/{fr_idx}.png")

            # save to file
            f_path = os.path.join(
                allf.trks_dir,
                allf.seqs[seq_idx],
                allf.frames[seq_idx][fr_idx].replace("nby", "txt"),
            )
            os.makedirs(os.path.dirname(f_path), exist_ok=True)
            np.savetxt(
                f_path, trks, delimiter=",",
            )
