// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/JointState
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
#include "whole_body_state_msgs/JointState.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_JointState_common : public MATLABROSMsgInterface<whole_body_state_msgs::JointState> {
  public:
    virtual ~whole_body_state_msgs_msg_JointState_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::JointState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::JointState* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_JointState_common::copy_from_struct(whole_body_state_msgs::JointState* msg, const matlab::data::Struct& arr,
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
        //position
        const matlab::data::TypedArray<double> position_arr = arr["Position"];
        msg->position = position_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Position' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Position' is wrong type; expected a double.");
    }
    try {
        //velocity
        const matlab::data::TypedArray<double> velocity_arr = arr["Velocity"];
        msg->velocity = velocity_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Velocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Velocity' is wrong type; expected a double.");
    }
    try {
        //effort
        const matlab::data::TypedArray<double> effort_arr = arr["Effort"];
        msg->effort = effort_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Effort' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Effort' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_JointState_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::JointState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Name","Position","Velocity","Effort"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/JointState");
    // name
    auto currentElement_name = (msg + ctr)->name;
    outArray[ctr]["Name"] = factory.createCharArray(currentElement_name);
    // position
    auto currentElement_position = (msg + ctr)->position;
    outArray[ctr]["Position"] = factory.createScalar(currentElement_position);
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    outArray[ctr]["Velocity"] = factory.createScalar(currentElement_velocity);
    // effort
    auto currentElement_effort = (msg + ctr)->effort;
    outArray[ctr]["Effort"] = factory.createScalar(currentElement_effort);
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_JointState_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_JointState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_JointState_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::JointState,whole_body_state_msgs_msg_JointState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_JointState_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::JointState,whole_body_state_msgs::JointState::ConstPtr,whole_body_state_msgs_msg_JointState_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_JointState_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::JointState,whole_body_state_msgs_msg_JointState_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_JointState_common, MATLABROSMsgInterface<whole_body_state_msgs::JointState>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_JointState_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1