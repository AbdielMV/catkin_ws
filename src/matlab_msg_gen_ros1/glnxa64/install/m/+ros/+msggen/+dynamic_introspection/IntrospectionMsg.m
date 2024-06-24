
classdef IntrospectionMsg < ros.Message
    %IntrospectionMsg MATLAB implementation of dynamic_introspection/IntrospectionMsg
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'dynamic_introspection/IntrospectionMsg' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'abf14c63c888d80e823c2b0710f2d3a3' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Bools' 'Doubles' 'Ints' 'Markers' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'bools' 'doubles' 'ints' 'markers' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.dynamic_introspection.BoolParameter' ...
            'ros.msggen.dynamic_introspection.DoubleParameter' ...
            'ros.msggen.dynamic_introspection.IntParameter' ...
            'ros.msggen.dynamic_introspection.MarkerParameter' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Bools
        Doubles
        Ints
        Markers
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'IntrospectionMsg', 'Header')
            obj.Header = val;
        end
        function set.Bools(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.dynamic_introspection.BoolParameter.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.dynamic_introspection.BoolParameter'};
            validateattributes(val, validClasses, validAttributes, 'IntrospectionMsg', 'Bools')
            obj.Bools = val;
        end
        function set.Doubles(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.dynamic_introspection.DoubleParameter.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.dynamic_introspection.DoubleParameter'};
            validateattributes(val, validClasses, validAttributes, 'IntrospectionMsg', 'Doubles')
            obj.Doubles = val;
        end
        function set.Ints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.dynamic_introspection.IntParameter.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.dynamic_introspection.IntParameter'};
            validateattributes(val, validClasses, validAttributes, 'IntrospectionMsg', 'Ints')
            obj.Ints = val;
        end
        function set.Markers(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.dynamic_introspection.MarkerParameter.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.dynamic_introspection.MarkerParameter'};
            validateattributes(val, validClasses, validAttributes, 'IntrospectionMsg', 'Markers')
            obj.Markers = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.dynamic_introspection.IntrospectionMsg.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.dynamic_introspection.IntrospectionMsg(strObj);
        end
    end
end