function [data, info] = jointCommand
%JointCommand gives an empty data for whole_body_state_msgs/JointCommand
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'whole_body_state_msgs/JointCommand';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Total, info.Total] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Feedforward, info.Feedforward] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Feedback, info.Feedback] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'whole_body_state_msgs/JointCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'total';
info.MatPath{3} = 'feedforward';
info.MatPath{4} = 'feedback';
