// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for twist_mux_msgs/JoyTurboActionResult
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
#include "twist_mux_msgs/JoyTurboActionResult.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_msg_JoyTurboActionResult_common : public MATLABROSMsgInterface<twist_mux_msgs::JoyTurboActionResult> {
  public:
    virtual ~twist_mux_msgs_msg_JoyTurboActionResult_common(){}
    virtual void copy_from_struct(twist_mux_msgs::JoyTurboActionResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboActionResult* msg, MultiLibLoader loader, size_t size = 1);
};
  void twist_mux_msgs_msg_JoyTurboActionResult_common::copy_from_struct(twist_mux_msgs::JoyTurboActionResult* msg, const matlab::data::Struct& arr,
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
        //status
        const matlab::data::StructArray status_arr = arr["Status"];
        auto msgClassPtr_status = getCommonObject<actionlib_msgs::GoalStatus>("actionlib_msgs_msg_GoalStatus_common",loader);
        msgClassPtr_status->copy_from_struct(&msg->status,status_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Status' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Status' is wrong type; expected a struct.");
    }
    try {
        //result
        const matlab::data::StructArray result_arr = arr["Result"];
        auto msgClassPtr_result = getCommonObject<twist_mux_msgs::JoyTurboResult>("twist_mux_msgs_msg_JoyTurboResult_common",loader);
        msgClassPtr_result->copy_from_struct(&msg->result,result_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Result' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Result' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T twist_mux_msgs_msg_JoyTurboActionResult_common::get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboActionResult* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Status","Result"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("twist_mux_msgs/JoyTurboActionResult");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // status
    auto currentElement_status = (msg + ctr)->status;
    auto msgClassPtr_status = getCommonObject<actionlib_msgs::GoalStatus>("actionlib_msgs_msg_GoalStatus_common",loader);
    outArray[ctr]["Status"] = msgClassPtr_status->get_arr(factory, &currentElement_status, loader);
    // result
    auto currentElement_result = (msg + ctr)->result;
    auto msgClassPtr_result = getCommonObject<twist_mux_msgs::JoyTurboResult>("twist_mux_msgs_msg_JoyTurboResult_common",loader);
    outArray[ctr]["Result"] = msgClassPtr_result->get_arr(factory, &currentElement_result, loader);
    }
    return std::move(outArray);
  } 
class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_JoyTurboActionResult_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~twist_mux_msgs_JoyTurboActionResult_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          twist_mux_msgs_JoyTurboActionResult_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<twist_mux_msgs::JoyTurboActionResult,twist_mux_msgs_msg_JoyTurboActionResult_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         twist_mux_msgs_JoyTurboActionResult_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<twist_mux_msgs::JoyTurboActionResult,twist_mux_msgs::JoyTurboActionResult::ConstPtr,twist_mux_msgs_msg_JoyTurboActionResult_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         twist_mux_msgs_JoyTurboActionResult_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<twist_mux_msgs::JoyTurboActionResult,twist_mux_msgs_msg_JoyTurboActionResult_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_msg_JoyTurboActionResult_common, MATLABROSMsgInterface<twist_mux_msgs::JoyTurboActionResult>)
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_JoyTurboActionResult_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1