% Auto-generated.  Do not edit!

% msg = harlie_base_Sonar()
%
% Sonar message type, fields include:
%   roslib_Header header
% single dist

% //! \htmlinclude Sonar.msg.html
function msg = harlie_base_Sonar()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/opt/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.dist = single(0);
msg.md5sum_ = @harlie_base_Sonar___md5sum;
msg.type_ = @harlie_base_Sonar___type;
msg.serializationLength_ = @harlie_base_Sonar___serializationLength;
msg.serialize_ = @harlie_base_Sonar___serialize;
msg.deserialize_ = @harlie_base_Sonar___deserialize;
msg.message_definition_ = @harlie_base_Sonar___message_definition;

function x = harlie_base_Sonar___md5sum()
x = '32a7fd24a5630b5643e0d1882893197a';

function x = harlie_base_Sonar___message_definition()
x = [    'Header header\n' ...
    'float32 dist\n' ...
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

function x = harlie_base_Sonar___type()
x = 'harlie_base/Sonar';

function l__ = harlie_base_Sonar___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 4;

function dat__ = harlie_base_Sonar___serialize(msg__, seq__, fid__)
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
c__ = c__ + fwrite(fid__, msg__.dist, 'single');
if( c__ ~= 1 )
    error('some members of msg harlie_base:Sonar are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = harlie_base_Sonar___deserialize(dat__, fid__)
msg__ = harlie_base_Sonar();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
msg__.dist = fread(fid__,1,'single');
if( file_created__ )
    fclose(fid__);
end
function l__ = harlie_base_Sonar___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

