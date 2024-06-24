
classdef WholeBodyTrajectory < ros.Message
    %WholeBodyTrajectory MATLAB implementation of whole_body_state_msgs/WholeBodyTrajectory
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'whole_body_state_msgs/WholeBodyTrajectory' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'db7500cf41f995e321b048c09318d860' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Actual' 'Trajectory' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'actual' 'trajectory' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.whole_body_state_msgs.WholeBodyState' ...
            'ros.msggen.whole_body_state_msgs.WholeBodyState' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Actual
        Trajectory
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyTrajectory', 'Header')
            obj.Header = val;
        end
        function set.Actual(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.whole_body_state_msgs.WholeBodyState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyTrajectory', 'Actual')
            obj.Actual = val;
        end
        function set.Trajectory(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.whole_body_state_msgs.WholeBodyState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.whole_body_state_msgs.WholeBodyState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyTrajectory', 'Trajectory')
            obj.Trajectory = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.whole_body_state_msgs.WholeBodyTrajectory.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.whole_body_state_msgs.WholeBodyTrajectory(strObj);
        end
    end
end