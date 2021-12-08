#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   notebook_util.py
@Date created  :   2021/11/24
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides categorical plot and customized folder traverse functions
for better notebook use.
"""
# =============================================================================

import os
import numpy as np
import os.path as path
import matplotlib
from matplotlib.patches import PathPatch
import matplotlib.pyplot as plt

plt.ioff()
import seaborn as sns


def set_box_color(bp, color):
    """
    Input:
        bp: boxplot instance
    """
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


def adjust_box_widths(g, fac):
    """Adjust the withs of a seaborn-generated boxplot."""
    ##iterating through Axes instances
    for ax in g.axes.flatten():

        ##iterating through axes artists:
        for c in ax.get_children():

            ##searching for PathPatches
            if isinstance(c, PathPatch):
                ##getting current width of box:
                p = c.get_path()
                verts = p.vertices
                verts_sub = verts[:-1]
                xmin = np.min(verts_sub[:, 0])
                xmax = np.max(verts_sub[:, 0])
                xmid = 0.5 * (xmin + xmax)
                xhalf = 0.5 * (xmax - xmin)

                ##setting new width of box
                xmin_new = xmid - fac * xhalf
                xmax_new = xmid + fac * xhalf
                verts_sub[verts_sub[:, 0] == xmin, 0] = xmin_new
                verts_sub[verts_sub[:, 0] == xmax, 0] = xmax_new

                ##setting new width of median line
                for l in ax.lines:
                    if np.all(l.get_xdata() == [xmin, xmax]):
                        l.set_xdata([xmin_new, xmax_new])


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


def violinplot(axes, df, metric, catogory, title, ylabel, ylim):

    sns.violinplot(x=catogory, y=metric, data=df, ax=axes)

    axes.yaxis.grid(True)

    axes.set_title(title)
    axes.set_ylabel(ylabel)
    axes.set_ylim(bottom=ylim[0], top=ylim[1])

    plt.show()


def categorical_plot(
    axes,
    df,
    metric,
    catogory,
    title,
    xlabel,
    ylabel,
    ylim,
    group=None,
    lgd_labels=None,
    loc='lower right',
    kind='violin',
    titlefontsz=12,
):

    sns.set_theme(style="whitegrid")

    # fmt: off
    # use stripplot (less points) instead of swarmplot to handle many datapoints
    sns.swarmplot(x=catogory, y=metric, hue=group, data=df, ax=axes,
                  size=6, alpha=0.8, palette="colorblind",
                  edgecolor='black', dodge=True,
                 )
    if kind == 'violin':
        sns.violinplot(x=catogory, y=metric, hue=group, data=df, ax=axes,
                       linewidth=1.1, notch=False, orient="v",
                       dodge=True, palette="pastel", inner=None,
                      )
    elif kind == 'box':
        sns.boxplot(x=catogory, y=metric, hue=group, data=df, ax=axes,
                    linewidth=2, notch=False, orient="v",
                    dodge=True, palette="pastel",
                   )

    # sns.despine(trim=True)

    # fmt: on
    axes.yaxis.grid(True)

    if group:
        # deduplicate labels
        # method1: https://stackoverflow.com/a/33440601/7961693
        # hand, labl = ax.get_legend_handles_labels()
        # plt.legend(np.unique(labl))

        # method2: https://stackoverflow.com/a/33424628/7961693
        lablout, handout = [], []
        hand, labl = axes.get_legend_handles_labels()
        for h, l in zip(hand, labl):
            if l not in lablout:
                lablout.append(l)
                handout.append(h)
        if lgd_labels:
            axes.legend(handles=handout, labels=lgd_labels, loc=loc)
        else:
            axes.legend(handles=handout, labels=lablout, loc=loc)

    axes.set_title(title, fontweight='bold', fontsize=titlefontsz)
    axes.set_xlabel(xlabel)
    axes.set_ylabel(ylabel)
    axes.set_ylim(bottom=ylim[0], top=ylim[1])
