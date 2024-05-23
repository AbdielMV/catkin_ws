function [data, info] = tfStreamGoal
%TfStreamGoal gives an empty data for tf_lookup/TfStreamGoal
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'tf_lookup/TfStreamGoal';
[data.Transforms, info.Transforms] = ros.internal.ros.custommessages.tf_lookup.subscription;
info.Transforms.MLdataType = 'struct';
info.Transforms.MaxLen = NaN;
info.Transforms.MinLen = 0;
data.Transforms = data.Transforms([],1);
[data.SubscriptionId, info.SubscriptionId] = ros.internal.ros.messages.ros.char('string',0);
[data.Update, info.Update] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'tf_lookup/TfStreamGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'transforms';
info.MatPath{2} = 'transforms.target';
info.MatPath{3} = 'transforms.source';
info.MatPath{4} = 'subscription_id';
info.MatPath{5} = 'update';
