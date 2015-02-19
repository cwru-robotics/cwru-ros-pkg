# rosbag_to_matlab

This package contains utility programs for getting rosbag files into matlab.

There are currently 2 ways of doing this: csv and .mat files.

Converting to CSVs is now the least preferred method of getting rosbag files into matlab.  The .mat file converter is significantly faster on both the conversion to a .mat file and loading into matlab/octave.

# MAT

Use the `bag_to_mat.py` script to convert a rosbag into a set of .mat files. The script creates a single .mat file for each topic. So a bag with 20 topics will create 20 mat files if all are converted successfully.

To run:
`rosrun rosbag_to_matlab bag_to_mat.py <path to bag file> <output directory>`

`<path to bag file>` is the path to the bag file you wish to convert

`<output directory>` is the path to the directory where the .mat files will be written.

## Known Issues
The code implementing the mat filewriter is very picky on the structures it will write to a file.  As such, many of the more complex data types will often cause the write to fail.  The program will warn the user in such an event.

### .mat file structure
The mat file converter creates mat files with two data matricies: a header and the data fields.  There is a one-to-one correspondance between the columns of the header matrix and the data matrix, that is, each element in the header matrix describes the corresponding column of elements in the data matrix. For example, if element 103 in the header matrix is "position.27", the position of the right wrist, column 103 in the data matrix contains all recorded positions of the right wrist. 

#### TODO:
Improve the failure rate on the mat filewriter for complex data types.

# CSV

This method is outdated. Only use this when the .mat converter doesn't convert your file.

---

Using the `bag_to_csv.sh` script one can convert a rosbag to a set of csv files that matlab
can not so easily import.

To use run the following `rosrun rosbag_to_matlab bag_to_csv.sh <bag location> <output location>`
where `<bag location>` is the path to the bag file you wish to convert and `<output location>` is the path
to where you would like the output files to go.

When run the script will create a directory in the `<output location>` with the same name as the input bag file.
Then it will parse the topics that are in the bag file. For each topic it will create a csv. The csv will live
in a directory under this bagfiles directory with the same name as the topic's namespace.

So for example if we have a bag file named `input.bag` and we want all of the files to go to the `csv_bag_output` folder we
would run the command `rosrun rosbag_to_matlab bag_to_csv.sh input.bag csv_bag_output`. If this bag file contains 2 topics,
`/one_dof/pump_info` and `/std_int`, a csv file for `/one_dof/pump_info` will be put in the `csv_bag_output/input/one_dof` directory with the
name `pump_info.csv`. `/std_int` will be put in the directory `csv_bag_output/input/` with the name `std_int.csv`.

The csv files have the column names at the top of file. Essentially every single field is output as a column. Arrays are serialized meaning that
this file format may not be ideal for complex types.



