// Copyright 2020-2021 The MathWorks, Inc.
// Common copy functions for twist_mux_msgs/JoyTurboGoal
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
#include "twist_mux_msgs/JoyTurboAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSActionTemplates.hpp"

class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_msg_JoyTurboGoal_common : public MATLABROSMsgInterface<twist_mux_msgs::JoyTurboGoal> {
  public:
    virtual ~twist_mux_msgs_msg_JoyTurboGoal_common(){}
    virtual void copy_from_struct(twist_mux_msgs::JoyTurboGoal* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboGoal* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void twist_mux_msgs_msg_JoyTurboGoal_common::copy_from_struct(twist_mux_msgs::JoyTurboGoal* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T twist_mux_msgs_msg_JoyTurboGoal_common::get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboGoal* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("twist_mux_msgs/JoyTurboGoal");
    }
    return std::move(outArray);
  }

class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_msg_JoyTurboResult_common : public MATLABROSMsgInterface<twist_mux_msgs::JoyTurboResult> {
  public:
    virtual ~twist_mux_msgs_msg_JoyTurboResult_common(){}
    virtual void copy_from_struct(twist_mux_msgs::JoyTurboResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboResult* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void twist_mux_msgs_msg_JoyTurboResult_common::copy_from_struct(twist_mux_msgs::JoyTurboResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T twist_mux_msgs_msg_JoyTurboResult_common::get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboResult* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("twist_mux_msgs/JoyTurboResult");
    }
    return std::move(outArray);
  }

class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_msg_JoyTurboFeedback_common : public MATLABROSMsgInterface<twist_mux_msgs::JoyTurboFeedback> {
  public:
    virtual ~twist_mux_msgs_msg_JoyTurboFeedback_common(){}
    virtual void copy_from_struct(twist_mux_msgs::JoyTurboFeedback* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboFeedback* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void twist_mux_msgs_msg_JoyTurboFeedback_common::copy_from_struct(twist_mux_msgs::JoyTurboFeedback* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T twist_mux_msgs_msg_JoyTurboFeedback_common::get_arr(MDFactory_T& factory, const twist_mux_msgs::JoyTurboFeedback* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("twist_mux_msgs/JoyTurboFeedback");
    }
    return std::move(outArray);
  }

class TWIST_MUX_MSGS_EXPORT twist_mux_msgs_JoyTurbo_action : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~twist_mux_msgs_JoyTurbo_action(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABActClientInterface> generateActClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          twist_mux_msgs_JoyTurbo_action::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSPublisherImpl<twist_mux_msgs::JoyTurboGoal,twist_mux_msgs_msg_JoyTurboGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSPublisherImpl<twist_mux_msgs::JoyTurboFeedback,twist_mux_msgs_msg_JoyTurboFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSPublisherImpl<twist_mux_msgs::JoyTurboResult,twist_mux_msgs_msg_JoyTurboResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         twist_mux_msgs_JoyTurbo_action::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSSubscriberImpl<twist_mux_msgs::JoyTurboGoal,twist_mux_msgs::JoyTurboGoal::ConstPtr,twist_mux_msgs_msg_JoyTurboGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSSubscriberImpl<twist_mux_msgs::JoyTurboFeedback,twist_mux_msgs::JoyTurboFeedback::ConstPtr,twist_mux_msgs_msg_JoyTurboFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSSubscriberImpl<twist_mux_msgs::JoyTurboResult,twist_mux_msgs::JoyTurboResult::ConstPtr,twist_mux_msgs_msg_JoyTurboResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABActClientInterface> 
          twist_mux_msgs_JoyTurbo_action::generateActClientInterface(){
      return std::make_shared<ROSActClientImpl<twist_mux_msgs::JoyTurboAction,twist_mux_msgs::JoyTurboGoal,twist_mux_msgs::JoyTurboFeedbackConstPtr,twist_mux_msgs::JoyTurboResultConstPtr,twist_mux_msgs_msg_JoyTurboGoal_common,twist_mux_msgs_msg_JoyTurboFeedback_common,twist_mux_msgs_msg_JoyTurboResult_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          twist_mux_msgs_JoyTurbo_action::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSBagWriterImpl<twist_mux_msgs::JoyTurboGoal,twist_mux_msgs_msg_JoyTurboGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSBagWriterImpl<twist_mux_msgs::JoyTurboFeedback,twist_mux_msgs_msg_JoyTurboFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSBagWriterImpl<twist_mux_msgs::JoyTurboResult,twist_mux_msgs_msg_JoyTurboResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_msg_JoyTurboGoal_common, MATLABROSMsgInterface<twist_mux_msgs::JoyTurboGoal>)
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_msg_JoyTurboFeedback_common, MATLABROSMsgInterface<twist_mux_msgs::JoyTurboFeedback>)
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_msg_JoyTurboResult_common, MATLABROSMsgInterface<twist_mux_msgs::JoyTurboResult>)
CLASS_LOADER_REGISTER_CLASS(twist_mux_msgs_JoyTurbo_action, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1