// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/CentroidalState
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
#include "whole_body_state_msgs/CentroidalState.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_CentroidalState_common : public MATLABROSMsgInterface<whole_body_state_msgs::CentroidalState> {
  public:
    virtual ~whole_body_state_msgs_msg_CentroidalState_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::CentroidalState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::CentroidalState* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_CentroidalState_common::copy_from_struct(whole_body_state_msgs::CentroidalState* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //com_position
        const matlab::data::StructArray com_position_arr = arr["ComPosition"];
        auto msgClassPtr_com_position = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_com_position->copy_from_struct(&msg->com_position,com_position_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ComPosition' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ComPosition' is wrong type; expected a struct.");
    }
    try {
        //com_velocity
        const matlab::data::StructArray com_velocity_arr = arr["ComVelocity"];
        auto msgClassPtr_com_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_com_velocity->copy_from_struct(&msg->com_velocity,com_velocity_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ComVelocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ComVelocity' is wrong type; expected a struct.");
    }
    try {
        //base_orientation
        const matlab::data::StructArray base_orientation_arr = arr["BaseOrientation"];
        auto msgClassPtr_base_orientation = getCommonObject<geometry_msgs::Quaternion>("geometry_msgs_msg_Quaternion_common",loader);
        msgClassPtr_base_orientation->copy_from_struct(&msg->base_orientation,base_orientation_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BaseOrientation' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BaseOrientation' is wrong type; expected a struct.");
    }
    try {
        //base_angular_velocity
        const matlab::data::StructArray base_angular_velocity_arr = arr["BaseAngularVelocity"];
        auto msgClassPtr_base_angular_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_base_angular_velocity->copy_from_struct(&msg->base_angular_velocity,base_angular_velocity_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BaseAngularVelocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BaseAngularVelocity' is wrong type; expected a struct.");
    }
    try {
        //momenta
        const matlab::data::StructArray momenta_arr = arr["Momenta"];
        auto msgClassPtr_momenta = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
        msgClassPtr_momenta->copy_from_struct(&msg->momenta,momenta_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Momenta' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Momenta' is wrong type; expected a struct.");
    }
    try {
        //momenta_rate
        const matlab::data::StructArray momenta_rate_arr = arr["MomentaRate"];
        auto msgClassPtr_momenta_rate = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
        msgClassPtr_momenta_rate->copy_from_struct(&msg->momenta_rate,momenta_rate_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'MomentaRate' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'MomentaRate' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_CentroidalState_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::CentroidalState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ComPosition","ComVelocity","BaseOrientation","BaseAngularVelocity","Momenta","MomentaRate"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/CentroidalState");
    // com_position
    auto currentElement_com_position = (msg + ctr)->com_position;
    auto msgClassPtr_com_position = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["ComPosition"] = msgClassPtr_com_position->get_arr(factory, &currentElement_com_position, loader);
    // com_velocity
    auto currentElement_com_velocity = (msg + ctr)->com_velocity;
    auto msgClassPtr_com_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["ComVelocity"] = msgClassPtr_com_velocity->get_arr(factory, &currentElement_com_velocity, loader);
    // base_orientation
    auto currentElement_base_orientation = (msg + ctr)->base_orientation;
    auto msgClassPtr_base_orientation = getCommonObject<geometry_msgs::Quaternion>("geometry_msgs_msg_Quaternion_common",loader);
    outArray[ctr]["BaseOrientation"] = msgClassPtr_base_orientation->get_arr(factory, &currentElement_base_orientation, loader);
    // base_angular_velocity
    auto currentElement_base_angular_velocity = (msg + ctr)->base_angular_velocity;
    auto msgClassPtr_base_angular_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["BaseAngularVelocity"] = msgClassPtr_base_angular_velocity->get_arr(factory, &currentElement_base_angular_velocity, loader);
    // momenta
    auto currentElement_momenta = (msg + ctr)->momenta;
    auto msgClassPtr_momenta = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
    outArray[ctr]["Momenta"] = msgClassPtr_momenta->get_arr(factory, &currentElement_momenta, loader);
    // momenta_rate
    auto currentElement_momenta_rate = (msg + ctr)->momenta_rate;
    auto msgClassPtr_momenta_rate = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
    outArray[ctr]["MomentaRate"] = msgClassPtr_momenta_rate->get_arr(factory, &currentElement_momenta_rate, loader);
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_CentroidalState_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_CentroidalState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_CentroidalState_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::CentroidalState,whole_body_state_msgs_msg_CentroidalState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_CentroidalState_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::CentroidalState,whole_body_state_msgs::CentroidalState::ConstPtr,whole_body_state_msgs_msg_CentroidalState_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_CentroidalState_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::CentroidalState,whole_body_state_msgs_msg_CentroidalState_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_CentroidalState_common, MATLABROSMsgInterface<whole_body_state_msgs::CentroidalState>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_CentroidalState_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1