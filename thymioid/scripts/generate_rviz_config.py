#!/usr/bin/env python

# import os
import sys


def main():
    path, name = sys.argv[1:3]
    t_path = '{path}.{name}'.format(**locals())
    # if os.path.exists(t_path):
    #     return
    with open(path) as f:
        config = f.read()
    # print(config)
    with open(t_path, 'w') as f:
        f.write(config.format(**locals()))


if __name__ == '__main__':
    main()
