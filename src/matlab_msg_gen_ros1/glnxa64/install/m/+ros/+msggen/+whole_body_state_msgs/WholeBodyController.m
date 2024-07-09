
classdef WholeBodyController < ros.Message
    %WholeBodyController MATLAB implementation of whole_body_state_msgs/WholeBodyController
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'whole_body_state_msgs/WholeBodyController' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '93439548158f55471ecd48dab55490cd' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Desired' 'Actual' 'Error' 'Command' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'desired' 'actual' 'error' 'command' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.whole_body_state_msgs.WholeBodyState' ...
            'ros.msggen.whole_body_state_msgs.WholeBodyState' ...
            'ros.msggen.whole_body_state_msgs.WholeBodyState' ...
            'ros.msggen.whole_body_state_msgs.JointCommand' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Desired
        Actual
        Error
        Command
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyController', 'Header')
            obj.Header = val;
        end
        function set.Desired(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.whole_body_state_msgs.WholeBodyState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyController', 'Desired')
            obj.Desired = val;
        end
        function set.Actual(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.whole_body_state_msgs.WholeBodyState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyController', 'Actual')
            obj.Actual = val;
        end
        function set.Error(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.whole_body_state_msgs.WholeBodyState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyController', 'Error')
            obj.Error = val;
        end
        function set.Command(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.whole_body_state_msgs.JointCommand.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.whole_body_state_msgs.JointCommand'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyController', 'Command')
            obj.Command = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.whole_body_state_msgs.WholeBodyController.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.whole_body_state_msgs.WholeBodyController(strObj);
        end
    end
end
