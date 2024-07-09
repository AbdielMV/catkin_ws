// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/WholeBodyTrajectory
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
#include "whole_body_state_msgs/WholeBodyTrajectory.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_WholeBodyTrajectory_common : public MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyTrajectory> {
  public:
    virtual ~whole_body_state_msgs_msg_WholeBodyTrajectory_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::WholeBodyTrajectory* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyTrajectory* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_WholeBodyTrajectory_common::copy_from_struct(whole_body_state_msgs::WholeBodyTrajectory* msg, const matlab::data::Struct& arr,
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
        //trajectory
        const matlab::data::StructArray trajectory_arr = arr["Trajectory"];
        for (auto _trajectoryarr : trajectory_arr) {
        	whole_body_state_msgs::WholeBodyState _val;
        auto msgClassPtr_trajectory = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
        msgClassPtr_trajectory->copy_from_struct(&_val,_trajectoryarr,loader);
        	msg->trajectory.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Trajectory' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Trajectory' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_WholeBodyTrajectory_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyTrajectory* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Actual","Trajectory"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/WholeBodyTrajectory");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // actual
    auto currentElement_actual = (msg + ctr)->actual;
    auto msgClassPtr_actual = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
    outArray[ctr]["Actual"] = msgClassPtr_actual->get_arr(factory, &currentElement_actual, loader);
    // trajectory
    auto currentElement_trajectory = (msg + ctr)->trajectory;
    auto msgClassPtr_trajectory = getCommonObject<whole_body_state_msgs::WholeBodyState>("whole_body_state_msgs_msg_WholeBodyState_common",loader);
    outArray[ctr]["Trajectory"] = msgClassPtr_trajectory->get_arr(factory,&currentElement_trajectory[0],loader,currentElement_trajectory.size());
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_WholeBodyTrajectory_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_WholeBodyTrajectory_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_WholeBodyTrajectory_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::WholeBodyTrajectory,whole_body_state_msgs_msg_WholeBodyTrajectory_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_WholeBodyTrajectory_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::WholeBodyTrajectory,whole_body_state_msgs::WholeBodyTrajectory::ConstPtr,whole_body_state_msgs_msg_WholeBodyTrajectory_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_WholeBodyTrajectory_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::WholeBodyTrajectory,whole_body_state_msgs_msg_WholeBodyTrajectory_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_WholeBodyTrajectory_common, MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyTrajectory>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_WholeBodyTrajectory_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1