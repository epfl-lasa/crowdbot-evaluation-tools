#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   file_io_util.py
@Date created  :   2022/02/22
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides utility functions for file i/o
"""
# =============================================================================
import pickle
import json


# save & load pickle
def save_dict2pkl(source_dict, pkl_path):
    with open(pkl_path, "wb") as filehandler:
        pickle.dump(source_dict, filehandler)


def load_pkl2dict(pkl_path):
    with open(pkl_path, 'rb') as filehandler:
        loaded_dict = pickle.load(filehandler)
    return loaded_dict


# save & load json
def save_dict2json(source_dict, json_path):
    with open(json_path, "w") as f:
        json.dump(source_dict, f, indent=2)


def load_json2dict(json_path):
    f = open(json_path)
    # convert string key into int
    loaded_dict = json.load(
        f,
        object_hook=lambda d: {
            int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()
        },
    )
    return loaded_dict
