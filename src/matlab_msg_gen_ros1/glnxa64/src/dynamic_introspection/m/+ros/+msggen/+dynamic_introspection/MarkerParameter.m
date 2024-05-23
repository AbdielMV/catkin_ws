
classdef MarkerParameter < ros.Message
    %MarkerParameter MATLAB implementation of dynamic_introspection/MarkerParameter
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'dynamic_introspection/MarkerParameter' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '320e3e3c740e97c435a8a239a215ff23' % The MD5 Checksum of the message definition
        PropertyList = { 'Value' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'value' 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.visualization_msgs.MarkerArray' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Value
        Name
    end
    methods
        function set.Value(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.visualization_msgs.MarkerArray'};
            validateattributes(val, validClasses, validAttributes, 'MarkerParameter', 'Value')
            obj.Value = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'MarkerParameter', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.dynamic_introspection.MarkerParameter.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.dynamic_introspection.MarkerParameter(strObj);
        end
    end
end
