
IP=`python FindIP.py`
echo "The found IP address was "
echo $IP
export ROS_IP=$IP
export ROS_HOSTNAME=$IP
export ROS_MASTER_URI=http://192.168.0.150:11311

