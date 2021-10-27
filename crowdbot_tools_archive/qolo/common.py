import os
import numpy as np

from transformation_utils import boxes_to_corners

# # for running on headerless server
# from pyvirtualdisplay import Display
# display = Display(visible=False, size=(1280, 1024))
# display.start()

from mayavi import mlab

import cv2


class AllFrames(object):
    def __init__(self, data_dir="./data/qolo_market_data"):
        self.lidar_dir = os.path.join(data_dir, "lidar")
        self.dets_dir = os.path.join(data_dir, "detections")
        self.trks_dir = os.path.join(data_dir, "tracks")

        self.seqs = os.listdir(self.lidar_dir)
        self.seqs.sort()
        self.frames = []
        for seq in self.seqs:
            frs = os.listdir(os.path.join(self.lidar_dir, seq))
            frs.sort()
            self.frames.append(frs)

    def nr_seqs(self):
        return len(self.seqs)

    def nr_frames(self, sq_idx):
        return len(self.frames[sq_idx])

    def __getitem__(self, sq_fr_idx):
        sq_idx, fr_idx = sq_fr_idx

        assert sq_idx < self.nr_seqs(), (
            "Sequence index out of range. "
            f"Requested {sq_idx}, maximum {self.nr_seqs()}."
        )
        assert fr_idx < self.nr_frames(sq_idx), (
            "Frame index out of range. "
            f"Requested {fr_idx}, maximum {self.nr_frames(sq_idx)}."
        )

        seq = self.seqs[sq_idx]
        fr = self.frames[sq_idx][fr_idx]

        l_path = os.path.join(self.lidar_dir, seq, fr)
        lidar = np.load(l_path) if os.path.isfile(l_path) else None
        lidar = lidar.T

        dets, dets_conf = None, None
        d_path = os.path.join(self.dets_dir, seq, fr.replace("nby", "txt"))
        if os.path.isfile(d_path):
            dets = np.loadtxt(d_path, dtype=np.float32, delimiter=",")
            dets_conf = dets[:, -1]  # sorted in descending order
            dets = dets[:, :-1]

        t_path = os.path.join(self.trks_dir, seq, fr.replace("nby", "txt"))
        trks = (
            np.loadtxt(t_path, dtype=np.float32, delimiter=",")
            if os.path.isfile(t_path)
            else None
        )

        return lidar, dets, dets_conf, trks


def id2color(id_):
    c_hsv = np.empty((1, 1, 3), dtype=np.float32)
    c_hsv[0, :, 0] = float((id_ * 33) % 360)
    c_hsv[0, :, 1] = 1
    c_hsv[0, :, 2] = 1
    c_bgr = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)[0]
    return tuple(*c_bgr)


def plot_frame(lidar, boxes, out_path=None):
    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    if out_path is not None:
        mlab.options.offscreen = True

    fig = mlab.figure(
        figure=None,
        bgcolor=(1, 1, 1),
        fgcolor=(0, 0, 0),
        engine=None,
        size=(1600, 1000),
    )

    mlab.points3d(
        lidar[0], lidar[1], lidar[2], scale_factor=0.05, color=gs_blue, figure=fig,
    )

    # plot detections
    if boxes.shape[1] == 7:
        corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)
        for corner_xyz in corners_xyz:
            for inds in connect_inds:
                mlab.plot3d(
                    corner_xyz[0, inds],
                    corner_xyz[1, inds],
                    corner_xyz[2, inds],
                    tube_radius=None,
                    line_width=3,
                    color=gs_yellow,
                    figure=fig,
                )
    # or tracks
    elif boxes.shape[1] == 8:
        ids = boxes[:, -1]
        boxes = boxes[:, :-1]
        corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)
        for id_, corner_xyz in zip(ids, corners_xyz):
            c = id2color(id_)
            for inds in connect_inds:
                mlab.plot3d(
                    corner_xyz[0, inds],
                    corner_xyz[1, inds],
                    corner_xyz[2, inds],
                    tube_radius=None,
                    line_width=3,
                    color=c,
                    figure=fig,
                )

    mlab.view(focalpoint=(0, 0, 0))
    mlab.move(50, 0, 5.0)
    # mlab.pitch(-10)

    if out_path is not None:
        mlab.savefig(out_path)
    else:
        mlab.show()

    return fig


if __name__ == "__main__":
    allf = AllFrames()
    for seq_idx in range(allf.nr_seqs()):
        seq = allf.seqs[seq_idx]
        for fr_idx in range(0, allf.nr_frames(seq_idx), 2):
            lidar, dets, dets_conf, trks = allf[seq_idx, fr_idx]
            fig = plot_frame(lidar, trks, f"./tmp/{fr_idx}.png")
            mlab.close(fig)

        break  # plot one sequence
