% Auto-generated.  Do not edit!

% msg = harlie_wsn_steering_DesiredState()
%
% DesiredState message type, fields include:
% single x
% single y
% single theta
% single rho

% //! \htmlinclude DesiredState.msg.html
function msg = harlie_wsn_steering_DesiredState()

msg = [];
msg.x = single(0);
msg.y = single(0);
msg.theta = single(0);
msg.rho = single(0);
msg.md5sum_ = @harlie_wsn_steering_DesiredState___md5sum;
msg.type_ = @harlie_wsn_steering_DesiredState___type;
msg.serializationLength_ = @harlie_wsn_steering_DesiredState___serializationLength;
msg.serialize_ = @harlie_wsn_steering_DesiredState___serialize;
msg.deserialize_ = @harlie_wsn_steering_DesiredState___deserialize;
msg.message_definition_ = @harlie_wsn_steering_DesiredState___message_definition;

function x = harlie_wsn_steering_DesiredState___md5sum()
x = 'e78e1c6935530c1507030b878e0bbe08';

function x = harlie_wsn_steering_DesiredState___message_definition()
x = [    'float32 x\n' ...
    'float32 y\n' ...
    'float32 theta\n' ...
    'float32 rho\n' ...
    '\n' ...
    '\n' ...
];

function x = harlie_wsn_steering_DesiredState___type()
x = 'harlie_wsn_steering/DesiredState';

function l__ = harlie_wsn_steering_DesiredState___serializationLength(msg)
l__ =  ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 4;

function dat__ = harlie_wsn_steering_DesiredState___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.x, 'single');
c__ = c__ + fwrite(fid__, msg__.y, 'single');
c__ = c__ + fwrite(fid__, msg__.theta, 'single');
c__ = c__ + fwrite(fid__, msg__.rho, 'single');
if( c__ ~= 4 )
    error('some members of msg harlie_wsn_steering:DesiredState are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = harlie_wsn_steering_DesiredState___deserialize(dat__, fid__)
msg__ = harlie_wsn_steering_DesiredState();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.x = fread(fid__,1,'single');
msg__.y = fread(fid__,1,'single');
msg__.theta = fread(fid__,1,'single');
msg__.rho = fread(fid__,1,'single');
if( file_created__ )
    fclose(fid__);
end
function l__ = harlie_wsn_steering_DesiredState___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

