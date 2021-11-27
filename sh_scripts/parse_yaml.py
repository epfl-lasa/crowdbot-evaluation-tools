# modify from https://github.com/cangers/misc/blob/master/parse_yaml.py (written in python2)

import yaml
import argparse
import os
import sys
import warnings
import string


class ParseYaml(object):
    """
    Script to read yaml file and create bash variable assignments as such:
    key1_key2..._keyn=value
    Which are written to stdout
    """

    # , sep, cap, set
    def __init__(self, file, sep='_', cap=False, prefix=None):
        self.file = file
        self.sep = sep
        self.cap = cap
        self.prefix = prefix
        self.data = self.parse_yaml(self.file)
        self.res = []

    def _walk_dict(self, indict, pre=None):
        """
        Main function to traverse the dictionary recursively.
        :param indict: the serialized yaml file
        :param pre: prefix to keep track of recursion depth
        :return: updates self.res in place.
        """
        for key, value in indict.items():
            if isinstance(value, dict):
                pre = pre + self.sep + key if pre is not None else key
                self._walk_dict(value, pre=pre)
            else:
                bash_var = pre + self.sep + key if pre is not None else key
                bash_var = (
                    self.prefix + self.sep + bash_var
                    if self.prefix is not None
                    else bash_var
                )
                bash_var = bash_var.upper() if self.cap else bash_var
                bash_assignment = "{}={}".format(bash_var, value)
                self.res.append(bash_assignment)

    def _set_res(self):
        """
        Entry point to traversing the dictionary.
        :return: updates self.res in place
        """
        self.res = []
        for key, value in self.data.items():
            if key is None:
                continue
            self._walk_dict({key: value})

    def parse_yaml(self, yaml_file):
        # yujie:need add `, Loader=yaml.FullLoader` in Python 3
        data = yaml.load(yaml_file, Loader=yaml.FullLoader)
        yaml_file.close()
        return data

    def to_stdout(self):
        self._set_res()
        msg = '\n'.join(self.res)
        print(msg)

    # get() will never be used at same time as to_stdout() when called from command line
    def get(self, key, default):
        self._set_res()
        val = [x.split('=')[1] for x in self.res if x.split('=')[0] == key]
        if len(val) > 1:
            raise ValueError('More than one value matched that key.')
        elif len(val) == 1:
            return val[0]
        elif len(val) == 0 and default is None:
            raise KeyError('Key not found and default not specified.')
        else:
            return default


# Make sure you don't use any characters that would screw with linux
def _valid_char(x):
    # yujie: change string.ascii_letters from string.letters
    alphanum = ''.join([str(i) for i in range(0, 10)]) + string.ascii_letters + '_'
    if x not in alphanum:
        raise ValueError("Invalid separator character.")
    return x


def _init_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="YAML file", type=argparse.FileType('r'))
    parser.add_argument(
        "--sep", help="Key-value separator", type=_valid_char, default='_'
    )
    parser.add_argument(
        "--cap",
        help="Capitalize variable(s)",
        type=bool,
        default=False,
        nargs='?',
        const=True,
    )
    parser.add_argument(
        "--prefix", help="Prefix for variable(s)", type=str, default=None
    )
    parser.add_argument("--get", help="Retrieve a value", type=str, default=None)
    parser.add_argument(
        "--default", help="Default value if key is not found (for --get)", default=None
    )
    return parser


def Main():
    parser = _init_args()
    args = parser.parse_args()
    if args.default is not None and args.get is None:
        warnings.warn("Argument --default ignored in absence of --get.")
    yml = ParseYaml(file=args.file, sep=args.sep, cap=args.cap, prefix=args.prefix)

    if args.get is not None:
        val = yml.get(args.get, args.default)
        # yujie: change print style in Python3
        print(val)
    else:
        yml.to_stdout()


if __name__ == '__main__':
    Main()
