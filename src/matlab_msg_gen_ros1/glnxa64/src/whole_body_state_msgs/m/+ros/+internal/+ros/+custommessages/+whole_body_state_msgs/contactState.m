function [data, info] = contactState
%ContactState gives an empty data for whole_body_state_msgs/ContactState
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'whole_body_state_msgs/ContactState';
[data.Locomotion, info.Locomotion] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.Manipulation, info.Manipulation] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.geometry_msgs.twist;
info.Velocity.MLdataType = 'struct';
[data.Wrench, info.Wrench] = ros.internal.ros.messages.geometry_msgs.wrench;
info.Wrench.MLdataType = 'struct';
[data.SurfaceNormal, info.SurfaceNormal] = ros.internal.ros.messages.geometry_msgs.vector3;
info.SurfaceNormal.MLdataType = 'struct';
[data.FrictionCoefficient, info.FrictionCoefficient] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'whole_body_state_msgs/ContactState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,37);
info.MatPath{1} = 'locomotion';
info.MatPath{2} = 'manipulation';
info.MatPath{3} = 'name';
info.MatPath{4} = 'type';
info.MatPath{5} = 'pose';
info.MatPath{6} = 'pose.position';
info.MatPath{7} = 'pose.position.x';
info.MatPath{8} = 'pose.position.y';
info.MatPath{9} = 'pose.position.z';
info.MatPath{10} = 'pose.orientation';
info.MatPath{11} = 'pose.orientation.x';
info.MatPath{12} = 'pose.orientation.y';
info.MatPath{13} = 'pose.orientation.z';
info.MatPath{14} = 'pose.orientation.w';
info.MatPath{15} = 'velocity';
info.MatPath{16} = 'velocity.linear';
info.MatPath{17} = 'velocity.linear.x';
info.MatPath{18} = 'velocity.linear.y';
info.MatPath{19} = 'velocity.linear.z';
info.MatPath{20} = 'velocity.angular';
info.MatPath{21} = 'velocity.angular.x';
info.MatPath{22} = 'velocity.angular.y';
info.MatPath{23} = 'velocity.angular.z';
info.MatPath{24} = 'wrench';
info.MatPath{25} = 'wrench.force';
info.MatPath{26} = 'wrench.force.x';
info.MatPath{27} = 'wrench.force.y';
info.MatPath{28} = 'wrench.force.z';
info.MatPath{29} = 'wrench.torque';
info.MatPath{30} = 'wrench.torque.x';
info.MatPath{31} = 'wrench.torque.y';
info.MatPath{32} = 'wrench.torque.z';
info.MatPath{33} = 'surface_normal';
info.MatPath{34} = 'surface_normal.x';
info.MatPath{35} = 'surface_normal.y';
info.MatPath{36} = 'surface_normal.z';
info.MatPath{37} = 'friction_coefficient';
