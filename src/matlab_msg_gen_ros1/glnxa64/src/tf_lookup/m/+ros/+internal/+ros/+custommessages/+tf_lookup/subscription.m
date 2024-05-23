function [data, info] = subscription
%Subscription gives an empty data for tf_lookup/Subscription
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'tf_lookup/Subscription';
[data.Target, info.Target] = ros.internal.ros.messages.ros.char('string',0);
[data.Source, info.Source] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'tf_lookup/Subscription';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'target';
info.MatPath{2} = 'source';
