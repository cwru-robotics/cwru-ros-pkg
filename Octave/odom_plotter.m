%example Octave/matlab program to interpret odom data from a converted bag file
%assume the file is named: odom.mat, and that the *.mat file is in the current directory (same directory as this program)
clear all
load exp_odom.mat

%assemble seconds and nanoseconds to get time stamps
%subtract off starting time, so starts from t=0
%note: if load additional files, will need to use a single reference time stamp to allign
%all data to a common start time
odom_secs=data(:,2);
odom_nsecs=data(:,3);
odom_time= odom_secs+odom_nsecs*0.000000001;
odom_start_time = odom_time(1)
odom_time = odom_time-odom_start_time;

odom_x = data(:,6);
odom_y = data(:,7);
odom_qz = data(:,11);
odom_qw = data(:,12);
odom_heading = 2.0*atan2(odom_qz,odom_qw);%cheap conversion from quaternion to heading for planar motion
vel = data(:,49);
omega = data(:,54);

%load another topic file:
load exp_cmd_vel_stamped.mat
cmd_vel_secs=data(:,2);
cmd_vel_nsecs=data(:,3);
cmd_vel_time= cmd_vel_secs+cmd_vel_nsecs*0.000000001;
cmd_vel_time = cmd_vel_time-odom_start_time; %offset relative to odom time, so time is aligned

cmd_vel = data(:,5);
cmd_omega = data(:,10);

figure(1)
plot(odom_time,odom_x,'r',odom_time,odom_y,'b',odom_time,odom_heading,'g')
xlabel('time (sec)')
ylabel('m, rad')
title('x (r), y (b) and heading (g) vs time from odom')
grid on

figure(2)
clf
plot(odom_time,vel,'b',odom_time,omega,'r')
hold on
plot(cmd_vel_time,cmd_vel,'g',cmd_vel_time,cmd_omega,'m')
xlabel('time (sec)')
ylabel('m/sec and rad/sec')
title('speed (b) and spin (r) vs time')
grid on

figure(3)
plot(odom_x,odom_y)
xlabel('x (m)')
ylabel('y (m)')
title('path: x vs y')
grid

