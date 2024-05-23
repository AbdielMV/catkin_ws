function [data, info] = tfStreamResult
%TfStreamResult gives an empty data for tf_lookup/TfStreamResult
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'tf_lookup/TfStreamResult';
[data.SubscriptionId, info.SubscriptionId] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'tf_lookup/TfStreamResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'subscription_id';
info.MatPath{2} = 'topic';
