function [data, info] = jointState
%JointState gives an empty data for whole_body_state_msgs/JointState
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'whole_body_state_msgs/JointState';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Effort, info.Effort] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'whole_body_state_msgs/JointState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'position';
info.MatPath{3} = 'velocity';
info.MatPath{4} = 'effort';
