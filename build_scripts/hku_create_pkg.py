#!/usr/bin/env python

import argparse
import functools
import getpass
import os
import sys

def error(msg):
    sys.stderr.write("error: %s\n" % msg)
    sys.exit(1)

parser = argparse.ArgumentParser(
    description = """
Create a ros package in the current directory using catkin-simple.
    """
)
parser.add_argument('name', help='name of package to create')
parser.add_argument('deps', nargs='*', help='dependencies of package')
args = parser.parse_args()

print("Creating package '" + args.name + "'")

if (os.path.exists('./' + args.name)):
    error("Directory '%s' already exists under current directory." % args.name)

os.makedirs(args.name)
os.chdir(args.name)
os.makedirs('src')
os.makedirs('include')


with open('README.md', 'w') as f:
    f.write("""# %s

Your description goes here

## Example usage

## Running tests/demos
    """ % args.name)

username = getpass.getuser()
build_depends = "\n".join(map(lambda x: "<build_depend>%s</build_depend>" % x, args.deps))
run_depends = "\n".join(map(lambda x: "<run_depend>%s</run_depend>" % x, args.deps))

with open('package.xml', 'w') as f:
    f.write("""<?xml version="1.0"?>
<package>
  <name>%s</name>
  <version>0.0.0</version>
  <description>The %s package</description>
  
  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="%s@todo.todo">%s</maintainer>

  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but mutiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://ros.org/wiki/jacobian_publisher</url> -->


  <!-- Author tags are optional, mutiple are allowed, one per tag -->
  <!-- Authors do not have to be maintianers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <buildtool_depend>catkin_simple</buildtool_depend>
  %s
  %s
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- You can specify that this package is a metapackage here: -->
    <!-- <metapackage/> -->

    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
    """ % (args.name, args.name, username, username, build_depends, run_depends))

with open('CMakeLists.txt', 'w') as f:
    f.write("""cmake_minimum_required(VERSION 2.8.3)
project(%s)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    """ % args.name)


print('Done.')

