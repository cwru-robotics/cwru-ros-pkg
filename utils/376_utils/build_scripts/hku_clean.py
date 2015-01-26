#!/usr/bin/env python

import argparse
import build_functions

parser = argparse.ArgumentParser(description='Delete target and intermediate files for selected packages')
parser.add_argument('packages', metavar='p', type=str, nargs='*', help='packages to clean')
parser.add_argument('--with-deps', '-d', action='store_true', help='clean dependencies also')
args = parser.parse_args()

ret = build_functions.hku_clean(args.packages, args.with_deps)

exit(0 if ret else -1)
