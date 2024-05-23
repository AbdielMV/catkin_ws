// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for tf_lookup/TfLookupAction
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
#include "tf_lookup/TfLookupAction.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class TF_LOOKUP_EXPORT tf_lookup_msg_TfLookupAction_common : public MATLABROSMsgInterface<tf_lookup::TfLookupAction> {
  public:
    virtual ~tf_lookup_msg_TfLookupAction_common(){}
    virtual void copy_from_struct(tf_lookup::TfLookupAction* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::TfLookupAction* msg, MultiLibLoader loader, size_t size = 1);
};
  void tf_lookup_msg_TfLookupAction_common::copy_from_struct(tf_lookup::TfLookupAction* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //action_goal
        const matlab::data::StructArray action_goal_arr = arr["ActionGoal"];
        auto msgClassPtr_action_goal = getCommonObject<tf_lookup::TfLookupActionGoal>("tf_lookup_msg_TfLookupActionGoal_common",loader);
        msgClassPtr_action_goal->copy_from_struct(&msg->action_goal,action_goal_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionGoal' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionGoal' is wrong type; expected a struct.");
    }
    try {
        //action_result
        const matlab::data::StructArray action_result_arr = arr["ActionResult"];
        auto msgClassPtr_action_result = getCommonObject<tf_lookup::TfLookupActionResult>("tf_lookup_msg_TfLookupActionResult_common",loader);
        msgClassPtr_action_result->copy_from_struct(&msg->action_result,action_result_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionResult' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionResult' is wrong type; expected a struct.");
    }
    try {
        //action_feedback
        const matlab::data::StructArray action_feedback_arr = arr["ActionFeedback"];
        auto msgClassPtr_action_feedback = getCommonObject<tf_lookup::TfLookupActionFeedback>("tf_lookup_msg_TfLookupActionFeedback_common",loader);
        msgClassPtr_action_feedback->copy_from_struct(&msg->action_feedback,action_feedback_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ActionFeedback' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ActionFeedback' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_TfLookupAction_common::get_arr(MDFactory_T& factory, const tf_lookup::TfLookupAction* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ActionGoal","ActionResult","ActionFeedback"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/TfLookupAction");
    // action_goal
    auto currentElement_action_goal = (msg + ctr)->action_goal;
    auto msgClassPtr_action_goal = getCommonObject<tf_lookup::TfLookupActionGoal>("tf_lookup_msg_TfLookupActionGoal_common",loader);
    outArray[ctr]["ActionGoal"] = msgClassPtr_action_goal->get_arr(factory, &currentElement_action_goal, loader);
    // action_result
    auto currentElement_action_result = (msg + ctr)->action_result;
    auto msgClassPtr_action_result = getCommonObject<tf_lookup::TfLookupActionResult>("tf_lookup_msg_TfLookupActionResult_common",loader);
    outArray[ctr]["ActionResult"] = msgClassPtr_action_result->get_arr(factory, &currentElement_action_result, loader);
    // action_feedback
    auto currentElement_action_feedback = (msg + ctr)->action_feedback;
    auto msgClassPtr_action_feedback = getCommonObject<tf_lookup::TfLookupActionFeedback>("tf_lookup_msg_TfLookupActionFeedback_common",loader);
    outArray[ctr]["ActionFeedback"] = msgClassPtr_action_feedback->get_arr(factory, &currentElement_action_feedback, loader);
    }
    return std::move(outArray);
  } 
class TF_LOOKUP_EXPORT tf_lookup_TfLookupAction_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~tf_lookup_TfLookupAction_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          tf_lookup_TfLookupAction_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<tf_lookup::TfLookupAction,tf_lookup_msg_TfLookupAction_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         tf_lookup_TfLookupAction_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<tf_lookup::TfLookupAction,tf_lookup::TfLookupAction::ConstPtr,tf_lookup_msg_TfLookupAction_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         tf_lookup_TfLookupAction_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<tf_lookup::TfLookupAction,tf_lookup_msg_TfLookupAction_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_TfLookupAction_common, MATLABROSMsgInterface<tf_lookup::TfLookupAction>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_TfLookupAction_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1