function [data, info] = tfLookupFeedback
%TfLookupFeedback gives an empty data for tf_lookup/TfLookupFeedback
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'tf_lookup/TfLookupFeedback';
info.MessageType = 'tf_lookup/TfLookupFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
