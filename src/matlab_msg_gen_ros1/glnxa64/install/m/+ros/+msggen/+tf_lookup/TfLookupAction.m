
classdef TfLookupAction < ros.Message
    %TfLookupAction MATLAB implementation of tf_lookup/TfLookupAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'tf_lookup/TfLookupAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e717cc4354bed30fd1a3a92c547936a5' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.tf_lookup.TfLookupActionGoal' ...
            'ros.msggen.tf_lookup.TfLookupActionResult' ...
            'ros.msggen.tf_lookup.TfLookupActionFeedback' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ActionGoal
        ActionResult
        ActionFeedback
    end
    methods
        function set.ActionGoal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.tf_lookup.TfLookupActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'TfLookupAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.tf_lookup.TfLookupActionResult'};
            validateattributes(val, validClasses, validAttributes, 'TfLookupAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.tf_lookup.TfLookupActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'TfLookupAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.tf_lookup.TfLookupAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.tf_lookup.TfLookupAction(strObj);
        end
    end
end