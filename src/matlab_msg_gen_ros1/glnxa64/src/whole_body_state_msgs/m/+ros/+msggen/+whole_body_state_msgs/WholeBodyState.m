
classdef WholeBodyState < ros.Message
    %WholeBodyState MATLAB implementation of whole_body_state_msgs/WholeBodyState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'whole_body_state_msgs/WholeBodyState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '87b5e441cb24282fcb38578f4d9b53ab' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Centroidal' 'Joints' 'Contacts' 'Rhonn' 'Time' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'centroidal' 'joints' 'contacts' 'rhonn' 'time' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.whole_body_state_msgs.CentroidalState' ...
            'ros.msggen.whole_body_state_msgs.JointState' ...
            'ros.msggen.whole_body_state_msgs.ContactState' ...
            'ros.msggen.whole_body_state_msgs.Rhonn' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Centroidal
        Joints
        Contacts
        Rhonn
        Time
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Header')
            obj.Header = val;
        end
        function set.Centroidal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.whole_body_state_msgs.CentroidalState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Centroidal')
            obj.Centroidal = val;
        end
        function set.Joints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.whole_body_state_msgs.JointState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.whole_body_state_msgs.JointState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Joints')
            obj.Joints = val;
        end
        function set.Contacts(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.whole_body_state_msgs.ContactState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.whole_body_state_msgs.ContactState'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Contacts')
            obj.Contacts = val;
        end
        function set.Rhonn(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.whole_body_state_msgs.Rhonn.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.whole_body_state_msgs.Rhonn'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Rhonn')
            obj.Rhonn = val;
        end
        function set.Time(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'WholeBodyState', 'Time');
            obj.Time = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.whole_body_state_msgs.WholeBodyState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.whole_body_state_msgs.WholeBodyState(strObj);
        end
    end
end
