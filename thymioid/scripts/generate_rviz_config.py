#!/usr/bin/env python3

import sys


def main() -> None:
    path, name = sys.argv[1:3]
    t_path = '{path}.{name}'.format(**locals())
    # if os.path.exists(t_path):
    #     return
    with open(path) as f:
        config = f.read()
    with open(t_path, 'w') as f:
        f.write(config.format(**locals()))


if __name__ == '__main__':
    main()
