
classdef TfStreamResult < ros.Message
    %TfStreamResult MATLAB implementation of tf_lookup/TfStreamResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'tf_lookup/TfStreamResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a8f3e325856c12da00435a78bf464739' % The MD5 Checksum of the message definition
        PropertyList = { 'SubscriptionId' 'Topic' } % List of non-constant message properties
        ROSPropertyList = { 'subscription_id' 'topic' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        SubscriptionId
        Topic
    end
    methods
        function set.SubscriptionId(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TfStreamResult', 'SubscriptionId');
            obj.SubscriptionId = char(val);
        end
        function set.Topic(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TfStreamResult', 'Topic');
            obj.Topic = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.tf_lookup.TfStreamResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.tf_lookup.TfStreamResult(strObj);
        end
    end
end