# -*-coding:utf-8 -*-
"""
@File    :   notebook_util.py
@Time    :   2021/11/24
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import os.path as path
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_theme()

# derived from https://stackoverflow.com/a/53380401/7961693
def walk(top, topdown=True, onerror=None, followlinks=False, maxdepth=None):
    islink, join, isdir = path.islink, path.join, path.isdir
    names = os.listdir(top)
    dirs, nondirs = [], []
    for name in names:
        if isdir(join(top, name)):
            dirs.append(name)
        else:
            nondirs.append(name)

    if topdown:
        yield top, dirs, nondirs

    if maxdepth is None or maxdepth > 1:
        for name in dirs:
            new_path = join(top, name)
            if followlinks or not islink(new_path):
                for x in walk(
                    new_path,
                    topdown,
                    onerror,
                    followlinks,
                    None if maxdepth is None else maxdepth - 1,
                ):
                    yield x
    if not topdown:
        yield top, dirs, nondirs


def boxplot(axes, df, metric, catogory, title, ylabel, ylim):

    sns.violinplot(x=catogory, y=metric, data=df, ax=axes)

    axes.yaxis.grid(True)

    axes.set_title('Crowd Density within 5 m of qolo')
    axes.set_ylabel('Density [1/$m^2$]')
    axes.set_ylim(bottom=ylim[0], top=ylim[1])

    plt.show()
