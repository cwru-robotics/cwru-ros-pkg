#!/usr/bin/env bash

function usage() {
    echo
    echo "Usage:"
    echo "bag_to_csv.sh <input bag> <output directory>"
    echo 
    echo "<input bag>        - the input bag file"
    echo "<output directory> - the directory to output the csv in"
    echo
    echo "e.g."
    echo "bag_to_csv.sh input.bag output_dir"
    echo
}

if [[ $# -ne 2 ]]; then
    echo "Invalid parameters"
    usage
fi

# make the output directory
output_dir=$2/$(basename $1 | sed 's/\..*$//')
mkdir -p $output_dir

# find where topics starts
topics_line=$(rosbag info $1 | grep -n "topics:" | cut -f1 -d:)

# get the number of lines
end_line=$(rosbag info $1 | wc -l)

# get the topic on the "topics:" line
topics[0]=$(rosbag info $1 | sed -n "${topics_line},${topics_line} p" | awk '{ print $2; }')

# get the rest of the topics
for (( i=$topics_line + 1; i <= $end_line; i++)); do
    topics[$i]=$(rosbag info $1 | sed -n "$i,$i p" | awk '{ print $1; }')
done

# for each topic
for topic in ${topics[@]}; do
    # create the csv output directory
    dir_name=$(dirname $topic)
    base_name=$(basename $topic)

    mkdir -p $output_dir/$dir_name

    echo "Creating csv: $output_dir/$dir_name/$base_name.csv"
    rostopic echo -b $1 -p $topic > $output_dir/$dir_name/$base_name.csv
done