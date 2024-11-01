
classdef JoyPriorityActionFeedback < ros.Message
    %JoyPriorityActionFeedback MATLAB implementation of twist_mux_msgs/JoyPriorityActionFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'twist_mux_msgs/JoyPriorityActionFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'aae20e09065c3809e8a8e87c4c8953fd' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Status' 'Feedback' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'status' 'feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.actionlib_msgs.GoalStatus' ...
            'ros.msggen.twist_mux_msgs.JoyPriorityFeedback' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Status
        Feedback
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'JoyPriorityActionFeedback', 'Header')
            obj.Header = val;
        end
        function set.Status(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_msgs.GoalStatus'};
            validateattributes(val, validClasses, validAttributes, 'JoyPriorityActionFeedback', 'Status')
            obj.Status = val;
        end
        function set.Feedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.twist_mux_msgs.JoyPriorityFeedback'};
            validateattributes(val, validClasses, validAttributes, 'JoyPriorityActionFeedback', 'Feedback')
            obj.Feedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.twist_mux_msgs.JoyPriorityActionFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.twist_mux_msgs.JoyPriorityActionFeedback(strObj);
        end
    end
end
