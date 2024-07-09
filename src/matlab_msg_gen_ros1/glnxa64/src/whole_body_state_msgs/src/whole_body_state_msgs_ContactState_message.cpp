// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/ContactState
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
#include "whole_body_state_msgs/ContactState.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_ContactState_common : public MATLABROSMsgInterface<whole_body_state_msgs::ContactState> {
  public:
    virtual ~whole_body_state_msgs_msg_ContactState_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::ContactState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::ContactState* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_ContactState_common::copy_from_struct(whole_body_state_msgs::ContactState* msg, const matlab::data::Struct& arr,
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
        //type
        const matlab::data::TypedArray<uint8_t> type_arr = arr["Type"];
        msg->type = type_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Type' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Type' is wrong type; expected a uint8.");
    }
    try {
        //pose
        const matlab::data::StructArray pose_arr = arr["Pose"];
        auto msgClassPtr_pose = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
        msgClassPtr_pose->copy_from_struct(&msg->pose,pose_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Pose' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Pose' is wrong type; expected a struct.");
    }
    try {
        //velocity
        const matlab::data::StructArray velocity_arr = arr["Velocity"];
        auto msgClassPtr_velocity = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
        msgClassPtr_velocity->copy_from_struct(&msg->velocity,velocity_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Velocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Velocity' is wrong type; expected a struct.");
    }
    try {
        //wrench
        const matlab::data::StructArray wrench_arr = arr["Wrench"];
        auto msgClassPtr_wrench = getCommonObject<geometry_msgs::Wrench>("geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_wrench->copy_from_struct(&msg->wrench,wrench_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Wrench' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Wrench' is wrong type; expected a struct.");
    }
    try {
        //surface_normal
        const matlab::data::StructArray surface_normal_arr = arr["SurfaceNormal"];
        auto msgClassPtr_surface_normal = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_surface_normal->copy_from_struct(&msg->surface_normal,surface_normal_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SurfaceNormal' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SurfaceNormal' is wrong type; expected a struct.");
    }
    try {
        //friction_coefficient
        const matlab::data::TypedArray<double> friction_coefficient_arr = arr["FrictionCoefficient"];
        msg->friction_coefficient = friction_coefficient_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FrictionCoefficient' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FrictionCoefficient' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_ContactState_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::ContactState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Locomotion","Manipulation","Name","Type","Pose","Velocity","Wrench","SurfaceNormal","FrictionCoefficient"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/ContactState");
    // locomotion
    auto currentElement_locomotion = (msg + ctr)->locomotion;
    outArray[ctr]["Locomotion"] = factory.createScalar(static_cast<uint8_t>(currentElement_locomotion));
    // manipulation
    auto currentElement_manipulation = (msg + ctr)->manipulation;
    outArray[ctr]["Manipulation"] = factory.createScalar(static_cast<uint8_t>(currentElement_manipulation));
    // name
    auto currentElement_name = (msg + ctr)->name;
    outArray[ctr]["Name"] = factory.createCharArray(currentElement_name);
    // type
    auto currentElement_type = (msg + ctr)->type;
    outArray[ctr]["Type"] = factory.createScalar(currentElement_type);
    // pose
    auto currentElement_pose = (msg + ctr)->pose;
    auto msgClassPtr_pose = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
    outArray[ctr]["Pose"] = msgClassPtr_pose->get_arr(factory, &currentElement_pose, loader);
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    auto msgClassPtr_velocity = getCommonObject<geometry_msgs::Twist>("geometry_msgs_msg_Twist_common",loader);
    outArray[ctr]["Velocity"] = msgClassPtr_velocity->get_arr(factory, &currentElement_velocity, loader);
    // wrench
    auto currentElement_wrench = (msg + ctr)->wrench;
    auto msgClassPtr_wrench = getCommonObject<geometry_msgs::Wrench>("geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["Wrench"] = msgClassPtr_wrench->get_arr(factory, &currentElement_wrench, loader);
    // surface_normal
    auto currentElement_surface_normal = (msg + ctr)->surface_normal;
    auto msgClassPtr_surface_normal = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["SurfaceNormal"] = msgClassPtr_surface_normal->get_arr(factory, &currentElement_surface_normal, loader);
    // friction_coefficient
    auto currentElement_friction_coefficient = (msg + ctr)->friction_coefficient;
    outArray[ctr]["FrictionCoefficient"] = factory.createScalar(currentElement_friction_coefficient);
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_ContactState_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_ContactState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_ContactState_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::ContactState,whole_body_state_msgs_msg_ContactState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_ContactState_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::ContactState,whole_body_state_msgs::ContactState::ConstPtr,whole_body_state_msgs_msg_ContactState_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_ContactState_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::ContactState,whole_body_state_msgs_msg_ContactState_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_ContactState_common, MATLABROSMsgInterface<whole_body_state_msgs::ContactState>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_ContactState_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1