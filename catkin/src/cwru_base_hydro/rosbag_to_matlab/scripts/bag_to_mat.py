#!/usr/bin/env python

import sys
import os

#Add bag_to_mat_converter.py to the path so we can import the ROSBagConverter class.
pkg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../src')
sys.path.insert(0, pkg_path)

from bag_to_mat_converter import ROSBagConverter

if (__name__ == "__main__"):
    if not (len(sys.argv) > 2):
        print """
    Usage:
        bag_to_mat.py <input_bag_file> <output_directory>

        input_bag_file - bag file to convert to mat file
        output_directory - directory to write mat files

        e.g.
        bag_to_mat.py test.bag /my_awesome_directory_with_code_that_I_am_definitely_not_embarrassed_about
        """
        exit()
    try:
        bag_converter = ROSBagConverter(sys.argv[1], sys.argv[2])
    #Check the command line arguments
    except ValueError, e:
        print "Invalid arguments: {0}".format(str(e))
        exit()
    #Start the conversion
    if bag_converter.convert_bag_to_mat():
        print "Some files failed to be converted. Exiting..."
        exit()
    print "All files written successfully."
