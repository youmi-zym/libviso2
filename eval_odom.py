import copy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse

class EvalOdom(object):
    # ---------------------------------------------------
    # poses: N, 4, 4
    # pose: 4, 4
    # ---------------------------------------------------
    def __init__(self):
        self.lengths = [100, 200, 300, 400, 500]
        self.num_lengths = len(self.lengths)

    def loadPoses(self, file_name):
        with open(file_name, 'r') as fp:
            s = fp.readlines()
            poses = {}
            for cnt, line in enumerate(s):
                P = np.eye(4)
                line_split = [float(i) for i in line.split(" ")]
                withIdx = int(len(line_split) == 13)
                for row in range(3):
                    for col in range(4):
                        P[row, col] = line_split[row*4+col+withIdx]
                if withIdx:
                    frame_idx = int(line_split[0])
                else:
                    frame_idx = cnt
                poses[frame_idx] = P

        return poses

    def plotPath(self, seq, gt_poses, est_poses, draw_3d=True):
        plot_keys = ["Ground Truth", "Ours"]
        fontsize = 20
        plot_num = -1
        poses_dict = {}
        poses_dict["Ground Truth"] = gt_poses
        poses_dict["Ours"] = est_poses

        fig = plt.figure(figsize=(10, 10), dpi=80, tight_layout=True)
        if draw_3d:
            ax = fig.add_subplot(111, projection='3d')
            ax.set_aspect('auto')
            for key in plot_keys:
                pos_xyz = []
                for frame_idx in sorted(poses_dict[key].keys()):
                    pose = poses_dict[key][frame_idx]
                    pos_xyz.append([pose[0, 3], pose[1, 3], pose[2, 3]])
                pos_xyz = np.asarray(pos_xyz)

                ax.plot(pos_xyz[0], pos_xyz[1], pos_xyz[2])
            # png_title = "sequence3d_" + seq
            # plt.savefig('./' + png_title + '.pdf', bbox_inches='tight', pad_inches=0)
            plt.show()
        else: # 2d
            ax = plt.gca()
            ax.set_aspect('equal')
            for key in plot_keys:
                pos_xz = []
                for frame_idx in sorted(poses_dict[key].keys()):
                    pose = poses_dict[key][frame_idx]
                    pos_xz.append([pose[0, 3], pose[2, 3]])
                pos_xz = np.asarray(pos_xz)
                plt.plot(pos_xz[:, 0], pos_xz[:, 1], label=key)

            plt.legend(loc='upper left', prop={'size': fontsize})
            plt.xticks(fontsize=fontsize)
            plt.yticks(fontsize=fontsize)
            plt.xlabel('x (m)', fontsize=fontsize)
            plt.ylabel('z (m)', fontsize=fontsize)
            png_title = "sequence_"+seq
            plt.savefig('./'+png_title+'.pdf', bbox_inches='tight', pad_inches=0)

    def __call__(self, est_txt, gt_txt, seq=None, draw_3d=True):
        est_poses = self.loadPoses(est_txt)
        gt_poses = self.loadPoses(gt_txt)
        self.plotPath('00', gt_poses, est_poses, draw_3d=draw_3d)

if __name__ == '__main__':
    print("Eval Odom...")
    eval_odom = EvalOdom()
    # eval_odom('/mnt/Disk1/data/KITTI-VO/dataset/viso2/00.txt', '/mnt/Disk1/data/KITTI-VO/dataset/poses/00.txt',  draw_3d=False)
    eval_odom('/mnt/Disk1/data/KITTI-VO/dataset/viso2/00.txt', '/mnt/Disk1/data/KITTI-VO/dataset/viso2/00.txt',  draw_3d=False)

    print('Done!')


