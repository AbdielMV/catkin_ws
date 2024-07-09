
classdef lookupTransformRequest < ros.Message
    %lookupTransformRequest MATLAB implementation of tf_lookup/lookupTransformRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'tf_lookup/lookupTransformRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bb9d983758e61f286b43546ac9c0b080' % The MD5 Checksum of the message definition
        PropertyList = { 'TransformTime' 'TargetFrame' 'SourceFrame' } % List of non-constant message properties
        ROSPropertyList = { 'transform_time' 'target_frame' 'source_frame' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        TransformTime
        TargetFrame
        SourceFrame
    end
    methods
        function set.TransformTime(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'lookupTransformRequest', 'TransformTime')
            obj.TransformTime = val;
        end
        function set.TargetFrame(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'lookupTransformRequest', 'TargetFrame');
            obj.TargetFrame = char(val);
        end
        function set.SourceFrame(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'lookupTransformRequest', 'SourceFrame');
            obj.SourceFrame = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.tf_lookup.lookupTransformRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.tf_lookup.lookupTransformRequest(strObj);
        end
    end
end