function [data, info] = tfLookupGoal
%TfLookupGoal gives an empty data for tf_lookup/TfLookupGoal
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'tf_lookup/TfLookupGoal';
[data.TargetFrame, info.TargetFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.SourceFrame, info.SourceFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.TransformTime, info.TransformTime] = ros.internal.ros.messages.ros.time;
info.TransformTime.MLdataType = 'struct';
info.MessageType = 'tf_lookup/TfLookupGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'target_frame';
info.MatPath{2} = 'source_frame';
info.MatPath{3} = 'transform_time';
info.MatPath{4} = 'transform_time.sec';
info.MatPath{5} = 'transform_time.nsec';
