% Auto-generated.  Do not edit!

% msg = harlie_base_Pose()
%
% Pose message type, fields include:
%   roslib_Header header
% single x
% single y
% single theta
% single vel
% single omega
% single x_var
% single y_var
% single theta_var
% single vel_var
% single omega_var

% //! \htmlinclude Pose.msg.html
function msg = harlie_base_Pose()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/opt/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.x = single(0);
msg.y = single(0);
msg.theta = single(0);
msg.vel = single(0);
msg.omega = single(0);
msg.x_var = single(0);
msg.y_var = single(0);
msg.theta_var = single(0);
msg.vel_var = single(0);
msg.omega_var = single(0);
msg.md5sum_ = @harlie_base_Pose___md5sum;
msg.type_ = @harlie_base_Pose___type;
msg.serializationLength_ = @harlie_base_Pose___serializationLength;
msg.serialize_ = @harlie_base_Pose___serialize;
msg.deserialize_ = @harlie_base_Pose___deserialize;
msg.message_definition_ = @harlie_base_Pose___message_definition;

function x = harlie_base_Pose___md5sum()
x = '7ce3d88c4d07be05d54abf722834318f';

function x = harlie_base_Pose___message_definition()
x = [    'Header header\n' ...
    'float32 x\n' ...
    'float32 y\n' ...
    'float32 theta\n' ...
    'float32 vel\n' ...
    'float32 omega\n' ...
    'float32 x_var\n' ...
    'float32 y_var\n' ...
    'float32 theta_var\n' ...
    'float32 vel_var\n' ...
    'float32 omega_var\n' ...
    '\n' ...
    '================================================================================\n' ...
    'MSG: roslib/Header\n' ...
    '# Standard metadata for higher-level stamped data types.\n' ...
    '# This is generally used to communicate timestamped data \n' ...
    '# in a particular coordinate frame.\n' ...
    '# \n' ...
    '# sequence ID: consecutively increasing ID \n' ...
    'uint32 seq\n' ...
    '#Two-integer timestamp that is expressed as:\n' ...
    '# * stamp.secs: seconds (stamp_secs) since epoch\n' ...
    '# * stamp.nsecs: nanoseconds since stamp_secs\n' ...
    '# time-handling sugar is provided by the client library\n' ...
    'time stamp\n' ...
    '#Frame this data is associated with\n' ...
    '# 0: no frame\n' ...
    '# 1: global frame\n' ...
    'string frame_id\n' ...
    '\n' ...
    '\n' ...
];

function x = harlie_base_Pose___type()
x = 'harlie_base/Pose';

function l__ = harlie_base_Pose___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4;

function dat__ = harlie_base_Pose___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
if (msg__.header.seq == 0)
    msg__.header.seq = seq__;
end
if (msg__.header.stamp.sec == 0 && msg__.header.stamp.nsec == 0)
    msg__.header.stamp = rosoct_time_now();
end
msg__.header.serialize_(msg__.header, seq__, fid__);
c__ = c__ + fwrite(fid__, msg__.x, 'single');
c__ = c__ + fwrite(fid__, msg__.y, 'single');
c__ = c__ + fwrite(fid__, msg__.theta, 'single');
c__ = c__ + fwrite(fid__, msg__.vel, 'single');
c__ = c__ + fwrite(fid__, msg__.omega, 'single');
c__ = c__ + fwrite(fid__, msg__.x_var, 'single');
c__ = c__ + fwrite(fid__, msg__.y_var, 'single');
c__ = c__ + fwrite(fid__, msg__.theta_var, 'single');
c__ = c__ + fwrite(fid__, msg__.vel_var, 'single');
c__ = c__ + fwrite(fid__, msg__.omega_var, 'single');
if( c__ ~= 10 )
    error('some members of msg harlie_base:Pose are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = harlie_base_Pose___deserialize(dat__, fid__)
msg__ = harlie_base_Pose();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
msg__.x = fread(fid__,1,'single');
msg__.y = fread(fid__,1,'single');
msg__.theta = fread(fid__,1,'single');
msg__.vel = fread(fid__,1,'single');
msg__.omega = fread(fid__,1,'single');
msg__.x_var = fread(fid__,1,'single');
msg__.y_var = fread(fid__,1,'single');
msg__.theta_var = fread(fid__,1,'single');
msg__.vel_var = fread(fid__,1,'single');
msg__.omega_var = fread(fid__,1,'single');
if( file_created__ )
    fclose(fid__);
end
function l__ = harlie_base_Pose___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

