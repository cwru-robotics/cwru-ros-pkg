#!/usr/bin/env python

import argparse
import build_functions

parser = argparse.ArgumentParser(description='Make selected packages')
parser.add_argument('packages', metavar='p', type=str, nargs='*', help='packages to make')
parser.add_argument('--with-deps', '-d', action='store_true', help='make dependencies also')
parser.add_argument('--debug', action='store_true', help='build code in debug mode (no optimizations)')
args = parser.parse_args()

ret = build_functions.hku_make(args.packages, args.with_deps, args.debug)

exit(0 if ret else -1)
