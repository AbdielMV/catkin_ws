function [data, info] = introspectionMsg
%IntrospectionMsg gives an empty data for dynamic_introspection/IntrospectionMsg
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'dynamic_introspection/IntrospectionMsg';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Bools, info.Bools] = ros.internal.ros.custommessages.dynamic_introspection.boolParameter;
info.Bools.MLdataType = 'struct';
info.Bools.MaxLen = NaN;
info.Bools.MinLen = 0;
data.Bools = data.Bools([],1);
[data.Doubles, info.Doubles] = ros.internal.ros.custommessages.dynamic_introspection.doubleParameter;
info.Doubles.MLdataType = 'struct';
info.Doubles.MaxLen = NaN;
info.Doubles.MinLen = 0;
data.Doubles = data.Doubles([],1);
[data.Ints, info.Ints] = ros.internal.ros.custommessages.dynamic_introspection.intParameter;
info.Ints.MLdataType = 'struct';
info.Ints.MaxLen = NaN;
info.Ints.MinLen = 0;
data.Ints = data.Ints([],1);
[data.Markers, info.Markers] = ros.internal.ros.custommessages.dynamic_introspection.markerParameter;
info.Markers.MLdataType = 'struct';
info.Markers.MaxLen = NaN;
info.Markers.MinLen = 0;
data.Markers = data.Markers([],1);
info.MessageType = 'dynamic_introspection/IntrospectionMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,80);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'bools';
info.MatPath{8} = 'bools.name';
info.MatPath{9} = 'bools.value';
info.MatPath{10} = 'doubles';
info.MatPath{11} = 'doubles.name';
info.MatPath{12} = 'doubles.value';
info.MatPath{13} = 'ints';
info.MatPath{14} = 'ints.name';
info.MatPath{15} = 'ints.value';
info.MatPath{16} = 'markers';
info.MatPath{17} = 'markers.name';
info.MatPath{18} = 'markers.value';
info.MatPath{19} = 'markers.value.markers';
info.MatPath{20} = 'markers.value.markers.ARROW';
info.MatPath{21} = 'markers.value.markers.CUBE';
info.MatPath{22} = 'markers.value.markers.SPHERE';
info.MatPath{23} = 'markers.value.markers.CYLINDER';
info.MatPath{24} = 'markers.value.markers.LINE_STRIP';
info.MatPath{25} = 'markers.value.markers.LINE_LIST';
info.MatPath{26} = 'markers.value.markers.CUBE_LIST';
info.MatPath{27} = 'markers.value.markers.SPHERE_LIST';
info.MatPath{28} = 'markers.value.markers.POINTS';
info.MatPath{29} = 'markers.value.markers.TEXT_VIEW_FACING';
info.MatPath{30} = 'markers.value.markers.MESH_RESOURCE';
info.MatPath{31} = 'markers.value.markers.TRIANGLE_LIST';
info.MatPath{32} = 'markers.value.markers.ADD';
info.MatPath{33} = 'markers.value.markers.MODIFY';
info.MatPath{34} = 'markers.value.markers.DELETE';
info.MatPath{35} = 'markers.value.markers.DELETEALL';
info.MatPath{36} = 'markers.value.markers.header';
info.MatPath{37} = 'markers.value.markers.header.seq';
info.MatPath{38} = 'markers.value.markers.header.stamp';
info.MatPath{39} = 'markers.value.markers.header.stamp.sec';
info.MatPath{40} = 'markers.value.markers.header.stamp.nsec';
info.MatPath{41} = 'markers.value.markers.header.frame_id';
info.MatPath{42} = 'markers.value.markers.ns';
info.MatPath{43} = 'markers.value.markers.id';
info.MatPath{44} = 'markers.value.markers.type';
info.MatPath{45} = 'markers.value.markers.action';
info.MatPath{46} = 'markers.value.markers.pose';
info.MatPath{47} = 'markers.value.markers.pose.position';
info.MatPath{48} = 'markers.value.markers.pose.position.x';
info.MatPath{49} = 'markers.value.markers.pose.position.y';
info.MatPath{50} = 'markers.value.markers.pose.position.z';
info.MatPath{51} = 'markers.value.markers.pose.orientation';
info.MatPath{52} = 'markers.value.markers.pose.orientation.x';
info.MatPath{53} = 'markers.value.markers.pose.orientation.y';
info.MatPath{54} = 'markers.value.markers.pose.orientation.z';
info.MatPath{55} = 'markers.value.markers.pose.orientation.w';
info.MatPath{56} = 'markers.value.markers.scale';
info.MatPath{57} = 'markers.value.markers.scale.x';
info.MatPath{58} = 'markers.value.markers.scale.y';
info.MatPath{59} = 'markers.value.markers.scale.z';
info.MatPath{60} = 'markers.value.markers.color';
info.MatPath{61} = 'markers.value.markers.color.r';
info.MatPath{62} = 'markers.value.markers.color.g';
info.MatPath{63} = 'markers.value.markers.color.b';
info.MatPath{64} = 'markers.value.markers.color.a';
info.MatPath{65} = 'markers.value.markers.lifetime';
info.MatPath{66} = 'markers.value.markers.lifetime.sec';
info.MatPath{67} = 'markers.value.markers.lifetime.nsec';
info.MatPath{68} = 'markers.value.markers.frame_locked';
info.MatPath{69} = 'markers.value.markers.points';
info.MatPath{70} = 'markers.value.markers.points.x';
info.MatPath{71} = 'markers.value.markers.points.y';
info.MatPath{72} = 'markers.value.markers.points.z';
info.MatPath{73} = 'markers.value.markers.colors';
info.MatPath{74} = 'markers.value.markers.colors.r';
info.MatPath{75} = 'markers.value.markers.colors.g';
info.MatPath{76} = 'markers.value.markers.colors.b';
info.MatPath{77} = 'markers.value.markers.colors.a';
info.MatPath{78} = 'markers.value.markers.text';
info.MatPath{79} = 'markers.value.markers.mesh_resource';
info.MatPath{80} = 'markers.value.markers.mesh_use_embedded_materials';
