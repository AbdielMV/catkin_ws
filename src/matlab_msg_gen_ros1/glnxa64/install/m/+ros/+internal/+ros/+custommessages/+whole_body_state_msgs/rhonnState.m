function [data, info] = rhonnState
%RhonnState gives an empty data for whole_body_state_msgs/RhonnState
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'whole_body_state_msgs/RhonnState';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ErrorW1, info.ErrorW1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ErrorW2, info.ErrorW2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ObsPosition, info.ObsPosition] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ObsVelocity, info.ObsVelocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Reference, info.Reference] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'whole_body_state_msgs/RhonnState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'name';
info.MatPath{2} = 'position';
info.MatPath{3} = 'velocity';
info.MatPath{4} = 'error_w1';
info.MatPath{5} = 'error_w2';
info.MatPath{6} = 'obs_position';
info.MatPath{7} = 'obs_velocity';
info.MatPath{8} = 'reference';
