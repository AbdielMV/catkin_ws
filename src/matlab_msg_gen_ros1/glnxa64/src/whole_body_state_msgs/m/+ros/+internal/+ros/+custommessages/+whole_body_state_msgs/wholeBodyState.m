function [data, info] = wholeBodyState
%WholeBodyState gives an empty data for whole_body_state_msgs/WholeBodyState
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'whole_body_state_msgs/WholeBodyState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Time, info.Time] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Centroidal, info.Centroidal] = ros.internal.ros.custommessages.whole_body_state_msgs.centroidalState;
info.Centroidal.MLdataType = 'struct';
[data.Joints, info.Joints] = ros.internal.ros.custommessages.whole_body_state_msgs.jointState;
info.Joints.MLdataType = 'struct';
info.Joints.MaxLen = NaN;
info.Joints.MinLen = 0;
data.Joints = data.Joints([],1);
[data.Contacts, info.Contacts] = ros.internal.ros.custommessages.whole_body_state_msgs.contactState;
info.Contacts.MLdataType = 'struct';
info.Contacts.MaxLen = NaN;
info.Contacts.MinLen = 0;
data.Contacts = data.Contacts([],1);
[data.Rhonn, info.Rhonn] = ros.internal.ros.custommessages.whole_body_state_msgs.rhonn;
info.Rhonn.MLdataType = 'struct';
info.Rhonn.MaxLen = NaN;
info.Rhonn.MinLen = 0;
data.Rhonn = data.Rhonn([],1);
info.MessageType = 'whole_body_state_msgs/WholeBodyState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,93);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'time';
info.MatPath{8} = 'centroidal';
info.MatPath{9} = 'centroidal.com_position';
info.MatPath{10} = 'centroidal.com_position.x';
info.MatPath{11} = 'centroidal.com_position.y';
info.MatPath{12} = 'centroidal.com_position.z';
info.MatPath{13} = 'centroidal.com_velocity';
info.MatPath{14} = 'centroidal.com_velocity.x';
info.MatPath{15} = 'centroidal.com_velocity.y';
info.MatPath{16} = 'centroidal.com_velocity.z';
info.MatPath{17} = 'centroidal.base_orientation';
info.MatPath{18} = 'centroidal.base_orientation.x';
info.MatPath{19} = 'centroidal.base_orientation.y';
info.MatPath{20} = 'centroidal.base_orientation.z';
info.MatPath{21} = 'centroidal.base_orientation.w';
info.MatPath{22} = 'centroidal.base_angular_velocity';
info.MatPath{23} = 'centroidal.base_angular_velocity.x';
info.MatPath{24} = 'centroidal.base_angular_velocity.y';
info.MatPath{25} = 'centroidal.base_angular_velocity.z';
info.MatPath{26} = 'centroidal.momenta';
info.MatPath{27} = 'centroidal.momenta.linear';
info.MatPath{28} = 'centroidal.momenta.linear.x';
info.MatPath{29} = 'centroidal.momenta.linear.y';
info.MatPath{30} = 'centroidal.momenta.linear.z';
info.MatPath{31} = 'centroidal.momenta.angular';
info.MatPath{32} = 'centroidal.momenta.angular.x';
info.MatPath{33} = 'centroidal.momenta.angular.y';
info.MatPath{34} = 'centroidal.momenta.angular.z';
info.MatPath{35} = 'centroidal.momenta_rate';
info.MatPath{36} = 'centroidal.momenta_rate.linear';
info.MatPath{37} = 'centroidal.momenta_rate.linear.x';
info.MatPath{38} = 'centroidal.momenta_rate.linear.y';
info.MatPath{39} = 'centroidal.momenta_rate.linear.z';
info.MatPath{40} = 'centroidal.momenta_rate.angular';
info.MatPath{41} = 'centroidal.momenta_rate.angular.x';
info.MatPath{42} = 'centroidal.momenta_rate.angular.y';
info.MatPath{43} = 'centroidal.momenta_rate.angular.z';
info.MatPath{44} = 'joints';
info.MatPath{45} = 'joints.name';
info.MatPath{46} = 'joints.position';
info.MatPath{47} = 'joints.velocity';
info.MatPath{48} = 'joints.effort';
info.MatPath{49} = 'contacts';
info.MatPath{50} = 'contacts.locomotion';
info.MatPath{51} = 'contacts.manipulation';
info.MatPath{52} = 'contacts.name';
info.MatPath{53} = 'contacts.type';
info.MatPath{54} = 'contacts.pose';
info.MatPath{55} = 'contacts.pose.position';
info.MatPath{56} = 'contacts.pose.position.x';
info.MatPath{57} = 'contacts.pose.position.y';
info.MatPath{58} = 'contacts.pose.position.z';
info.MatPath{59} = 'contacts.pose.orientation';
info.MatPath{60} = 'contacts.pose.orientation.x';
info.MatPath{61} = 'contacts.pose.orientation.y';
info.MatPath{62} = 'contacts.pose.orientation.z';
info.MatPath{63} = 'contacts.pose.orientation.w';
info.MatPath{64} = 'contacts.velocity';
info.MatPath{65} = 'contacts.velocity.linear';
info.MatPath{66} = 'contacts.velocity.linear.x';
info.MatPath{67} = 'contacts.velocity.linear.y';
info.MatPath{68} = 'contacts.velocity.linear.z';
info.MatPath{69} = 'contacts.velocity.angular';
info.MatPath{70} = 'contacts.velocity.angular.x';
info.MatPath{71} = 'contacts.velocity.angular.y';
info.MatPath{72} = 'contacts.velocity.angular.z';
info.MatPath{73} = 'contacts.wrench';
info.MatPath{74} = 'contacts.wrench.force';
info.MatPath{75} = 'contacts.wrench.force.x';
info.MatPath{76} = 'contacts.wrench.force.y';
info.MatPath{77} = 'contacts.wrench.force.z';
info.MatPath{78} = 'contacts.wrench.torque';
info.MatPath{79} = 'contacts.wrench.torque.x';
info.MatPath{80} = 'contacts.wrench.torque.y';
info.MatPath{81} = 'contacts.wrench.torque.z';
info.MatPath{82} = 'contacts.surface_normal';
info.MatPath{83} = 'contacts.surface_normal.x';
info.MatPath{84} = 'contacts.surface_normal.y';
info.MatPath{85} = 'contacts.surface_normal.z';
info.MatPath{86} = 'contacts.friction_coefficient';
info.MatPath{87} = 'rhonn';
info.MatPath{88} = 'rhonn.name';
info.MatPath{89} = 'rhonn.position';
info.MatPath{90} = 'rhonn.velocity';
info.MatPath{91} = 'rhonn.error_w1';
info.MatPath{92} = 'rhonn.error_w2';
info.MatPath{93} = 'rhonn.effort';