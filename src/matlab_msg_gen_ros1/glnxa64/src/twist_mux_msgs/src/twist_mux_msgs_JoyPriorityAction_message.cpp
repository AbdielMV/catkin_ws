// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for twist_mux_msgs/JoyPriorityAction
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "twist_mux_msgs/JoyPriorityAction.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_msg_JoyPriorityAction_common : public MATLABROSMsgInterface<twist_mux_msgs::JoyPriorityAction> {
  public:
    virtual ~twist_mux_msgs_msg_JoyPriorityAction_common(){}
    virtual void copy_from_struct(twist_mux_msgs::JoyPriorityAction* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyPriorityAction* msg, MultiLibLoader loader, size_t size = 1);
};
  void twist_mux_msgs_msg_JoyPriorityAction_common::copy_from_struct(twist_mux_msgs::JoyPriorityAction* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //action_goal
        const matlab::data::StructArray action_goal_arr = arr["ActionGoal"];
        auto msgClassPtr_action_goal = getCommonObject<twist_mux_msgs::JoyPriorityActionGoal>("twist_mux_msgs_msg_JoyPriorityActionGoal_common",loader);
        msgClassPtr_action_goal->copy_from_struct(&msg->action_goal,action_goal_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionGoal' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionGoal' is wrong type; expected a struct.");
    }
    try {
        //action_result
        const matlab::data::StructArray action_result_arr = arr["ActionResult"];
        auto msgClassPtr_action_result = getCommonObject<twist_mux_msgs::JoyPriorityActionResult>("twist_mux_msgs_msg_JoyPriorityActionResult_common",loader);
        msgClassPtr_action_result->copy_from_struct(&msg->action_result,action_result_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionResult' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionResult' is wrong type; expected a struct.");
    }
    try {
        //action_feedback
        const matlab::data::StructArray action_feedback_arr = arr["ActionFeedback"];
        auto msgClassPtr_action_feedback = getCommonObject<twist_mux_msgs::JoyPriorityActionFeedback>("twist_mux_msgs_msg_JoyPriorityActionFeedback_common",loader);
        msgClassPtr_action_feedback->copy_from_struct(&msg->action_feedback,action_feedback_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionFeedback' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionFeedback' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T twist_mux_msgs_msg_JoyPriorityAction_common::get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyPriorityAction* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ActionGoal","ActionResult","ActionFeedback"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("twist_mux_msgs/JoyPriorityAction");
    // action_goal
    auto currentElement_action_goal = (msg + ctr)->action_goal;
    auto msgClassPtr_action_goal = getCommonObject<twist_mux_msgs::JoyPriorityActionGoal>("twist_mux_msgs_msg_JoyPriorityActionGoal_common",loader);
    outArray[ctr]["ActionGoal"] = msgClassPtr_action_goal->get_arr(factory, &currentElement_action_goal, loader);
    // action_result
    auto currentElement_action_result = (msg + ctr)->action_result;
    auto msgClassPtr_action_result = getCommonObject<twist_mux_msgs::JoyPriorityActionResult>("twist_mux_msgs_msg_JoyPriorityActionResult_common",loader);
    outArray[ctr]["ActionResult"] = msgClassPtr_action_result->get_arr(factory, &currentElement_action_result, loader);
    // action_feedback
    auto currentElement_action_feedback = (msg + ctr)->action_feedback;
    auto msgClassPtr_action_feedback = getCommonObject<twist_mux_msgs::JoyPriorityActionFeedback>("twist_mux_msgs_msg_JoyPriorityActionFeedback_common",loader);
    outArray[ctr]["ActionFeedback"] = msgClassPtr_action_feedback->get_arr(factory, &currentElement_action_feedback, loader);
    }
    return std::move(outArray);
  } 
class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_JoyPriorityAction_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~twist_mux_msgs_JoyPriorityAction_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          twist_mux_msgs_JoyPriorityAction_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<twist_mux_msgs::JoyPriorityAction,twist_mux_msgs_msg_JoyPriorityAction_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         twist_mux_msgs_JoyPriorityAction_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<twist_mux_msgs::JoyPriorityAction,twist_mux_msgs::JoyPriorityAction::ConstPtr,twist_mux_msgs_msg_JoyPriorityAction_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         twist_mux_msgs_JoyPriorityAction_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<twist_mux_msgs::JoyPriorityAction,twist_mux_msgs_msg_JoyPriorityAction_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_msg_JoyPriorityAction_common, MATLABROSMsgInterface<twist_mux_msgs::JoyPriorityAction>)
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_JoyPriorityAction_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1