// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/WholeBodyController
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
#include "whole_body_state_msgs/WholeBodyController.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_WholeBodyController_common : public MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyController> {
  public:
    virtual ~whole_body_state_msgs_msg_WholeBodyController_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::WholeBodyController* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyController* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_WholeBodyController_common::copy_from_struct(whole_body_state_msgs::WholeBodyController* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //desired
        const matlab::data::StructArray desired_arr = arr["Desired"];
        auto msgClassPtr_desired = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
        msgClassPtr_desired->copy_from_struct(&msg->desired,desired_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Desired' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Desired' is wrong type; expected a struct.");
    }
    try {
        //actual
        const matlab::data::StructArray actual_arr = arr["Actual"];
        auto msgClassPtr_actual = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
        msgClassPtr_actual->copy_from_struct(&msg->actual,actual_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Actual' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Actual' is wrong type; expected a struct.");
    }
    try {
        //error
        const matlab::data::StructArray error_arr = arr["Error"];
        auto msgClassPtr_error = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
        msgClassPtr_error->copy_from_struct(&msg->error,error_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Error' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Error' is wrong type; expected a struct.");
    }
    try {
        //command
        const matlab::data::StructArray command_arr = arr["Command"];
        for (auto _commandarr : command_arr) {
        	whole_body_state_msgs::JointCommand _val;
        auto msgClassPtr_command = getCommonObject<whole_body_state_msgs::JointCommand>("whole_body_state_msgs_msg_JointCommand_common",loader);
        msgClassPtr_command->copy_from_struct(&_val,_commandarr,loader);
        	msg->command.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Command' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Command' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_WholeBodyController_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyController* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Desired","Actual","Error","Command"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/WholeBodyController");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // desired
    auto currentElement_desired = (msg + ctr)->desired;
    auto msgClassPtr_desired = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
    outArray[ctr]["Desired"] = msgClassPtr_desired->get_arr(factory, &currentElement_desired, loader);
    // actual
    auto currentElement_actual = (msg + ctr)->actual;
    auto msgClassPtr_actual = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
    outArray[ctr]["Actual"] = msgClassPtr_actual->get_arr(factory, &currentElement_actual, loader);
    // error
    auto currentElement_error = (msg + ctr)->error;
    auto msgClassPtr_error = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
    outArray[ctr]["Error"] = msgClassPtr_error->get_arr(factory, &currentElement_error, loader);
    // command
    auto currentElement_command = (msg + ctr)->command;
    auto msgClassPtr_command = getCommonObject<whole_body_state_msgs::JointCommand>("whole_body_state_msgs_msg_JointCommand_common",loader);
    outArray[ctr]["Command"] = msgClassPtr_command->get_arr(factory,&currentElement_command[0],loader,currentElement_command.size());
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_WholeBodyController_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_WholeBodyController_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_WholeBodyController_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::WholeBodyController,whole_body_state_msgs_msg_WholeBodyController_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_WholeBodyController_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::WholeBodyController,whole_body_state_msgs::WholeBodyController::ConstPtr,whole_body_state_msgs_msg_WholeBodyController_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_WholeBodyController_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::WholeBodyController,whole_body_state_msgs_msg_WholeBodyController_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_WholeBodyController_common, MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyController>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_WholeBodyController_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1