// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/JointCommand
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
#include "whole_body_state_msgs/JointCommand.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_JointCommand_common : public MATLABROSMsgInterface<whole_body_state_msgs::JointCommand> {
  public:
    virtual ~whole_body_state_msgs_msg_JointCommand_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::JointCommand* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::JointCommand* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_JointCommand_common::copy_from_struct(whole_body_state_msgs::JointCommand* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //name
        const matlab::data::CharArray name_arr = arr["Name"];
        msg->name = name_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Name' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Name' is wrong type; expected a string.");
    }
    try {
        //total
        const matlab::data::TypedArray<double> total_arr = arr["Total"];
        msg->total = total_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Total' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Total' is wrong type; expected a double.");
    }
    try {
        //feedforward
        const matlab::data::TypedArray<double> feedforward_arr = arr["Feedforward"];
        msg->feedforward = feedforward_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Feedforward' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Feedforward' is wrong type; expected a double.");
    }
    try {
        //feedback
        const matlab::data::TypedArray<double> feedback_arr = arr["Feedback"];
        msg->feedback = feedback_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Feedback' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Feedback' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_JointCommand_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::JointCommand* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Name","Total","Feedforward","Feedback"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/JointCommand");
    // name
    auto currentElement_name = (msg + ctr)->name;
    outArray[ctr]["Name"] = factory.createCharArray(currentElement_name);
    // total
    auto currentElement_total = (msg + ctr)->total;
    outArray[ctr]["Total"] = factory.createScalar(currentElement_total);
    // feedforward
    auto currentElement_feedforward = (msg + ctr)->feedforward;
    outArray[ctr]["Feedforward"] = factory.createScalar(currentElement_feedforward);
    // feedback
    auto currentElement_feedback = (msg + ctr)->feedback;
    outArray[ctr]["Feedback"] = factory.createScalar(currentElement_feedback);
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_JointCommand_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_JointCommand_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_JointCommand_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::JointCommand,whole_body_state_msgs_msg_JointCommand_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_JointCommand_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::JointCommand,whole_body_state_msgs::JointCommand::ConstPtr,whole_body_state_msgs_msg_JointCommand_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_JointCommand_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::JointCommand,whole_body_state_msgs_msg_JointCommand_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_JointCommand_common, MATLABROSMsgInterface<whole_body_state_msgs::JointCommand>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_JointCommand_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1