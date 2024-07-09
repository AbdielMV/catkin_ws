// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for whole_body_state_msgs/WholeBodyState
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
#include "whole_body_state_msgs/WholeBodyState.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_msg_WholeBodyState_common : public MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyState> {
  public:
    virtual ~whole_body_state_msgs_msg_WholeBodyState_common(){}
    virtual void copy_from_struct(whole_body_state_msgs::WholeBodyState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyState* msg, MultiLibLoader loader, size_t size = 1);
};
  void whole_body_state_msgs_msg_WholeBodyState_common::copy_from_struct(whole_body_state_msgs::WholeBodyState* msg, const matlab::data::Struct& arr,
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
        //time
        const matlab::data::TypedArray<double> time_arr = arr["Time"];
        msg->time = time_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Time' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Time' is wrong type; expected a double.");
    }
    try {
        //centroidal
        const matlab::data::StructArray centroidal_arr = arr["Centroidal"];
        auto msgClassPtr_centroidal = getCommonObject<whole_body_state_msgs::CentroidalState>("whole_body_state_msgs_msg_CentroidalState_common",loader);
        msgClassPtr_centroidal->copy_from_struct(&msg->centroidal,centroidal_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Centroidal' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Centroidal' is wrong type; expected a struct.");
    }
    try {
        //joints
        const matlab::data::StructArray joints_arr = arr["Joints"];
        for (auto _jointsarr : joints_arr) {
        	whole_body_state_msgs::JointState _val;
        auto msgClassPtr_joints = getCommonObject<whole_body_state_msgs::JointState>("whole_body_state_msgs_msg_JointState_common",loader);
        msgClassPtr_joints->copy_from_struct(&_val,_jointsarr,loader);
        	msg->joints.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Joints' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Joints' is wrong type; expected a struct.");
    }
    try {
        //contacts
        const matlab::data::StructArray contacts_arr = arr["Contacts"];
        for (auto _contactsarr : contacts_arr) {
        	whole_body_state_msgs::ContactState _val;
        auto msgClassPtr_contacts = getCommonObject<whole_body_state_msgs::ContactState>("whole_body_state_msgs_msg_ContactState_common",loader);
        msgClassPtr_contacts->copy_from_struct(&_val,_contactsarr,loader);
        	msg->contacts.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Contacts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Contacts' is wrong type; expected a struct.");
    }
    try {
        //rhonn
        const matlab::data::StructArray rhonn_arr = arr["Rhonn"];
        for (auto _rhonnarr : rhonn_arr) {
        	whole_body_state_msgs::RhonnState _val;
        auto msgClassPtr_rhonn = getCommonObject<whole_body_state_msgs::RhonnState>("whole_body_state_msgs_msg_RhonnState_common",loader);
        msgClassPtr_rhonn->copy_from_struct(&_val,_rhonnarr,loader);
        	msg->rhonn.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Rhonn' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Rhonn' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T whole_body_state_msgs_msg_WholeBodyState_common::get_arr(MDFactory_T& factory, const whole_body_state_msgs::WholeBodyState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Time","Centroidal","Joints","Contacts","Rhonn"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("whole_body_state_msgs/WholeBodyState");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // time
    auto currentElement_time = (msg + ctr)->time;
    outArray[ctr]["Time"] = factory.createScalar(currentElement_time);
    // centroidal
    auto currentElement_centroidal = (msg + ctr)->centroidal;
    auto msgClassPtr_centroidal = getCommonObject<whole_body_state_msgs::CentroidalState>("whole_body_state_msgs_msg_CentroidalState_common",loader);
    outArray[ctr]["Centroidal"] = msgClassPtr_centroidal->get_arr(factory, &currentElement_centroidal, loader);
    // joints
    auto currentElement_joints = (msg + ctr)->joints;
    auto msgClassPtr_joints = getCommonObject<whole_body_state_msgs::JointState>("whole_body_state_msgs_msg_JointState_common",loader);
    outArray[ctr]["Joints"] = msgClassPtr_joints->get_arr(factory,&currentElement_joints[0],loader,currentElement_joints.size());
    // contacts
    auto currentElement_contacts = (msg + ctr)->contacts;
    auto msgClassPtr_contacts = getCommonObject<whole_body_state_msgs::ContactState>("whole_body_state_msgs_msg_ContactState_common",loader);
    outArray[ctr]["Contacts"] = msgClassPtr_contacts->get_arr(factory,&currentElement_contacts[0],loader,currentElement_contacts.size());
    // rhonn
    auto currentElement_rhonn = (msg + ctr)->rhonn;
    auto msgClassPtr_rhonn = getCommonObject<whole_body_state_msgs::RhonnState>("whole_body_state_msgs_msg_RhonnState_common",loader);
    outArray[ctr]["Rhonn"] = msgClassPtr_rhonn->get_arr(factory,&currentElement_rhonn[0],loader,currentElement_rhonn.size());
    }
    return std::move(outArray);
  } 
class WHOLE_BODY_STATE_MSGS_EXPORT whole_body_state_msgs_WholeBodyState_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~whole_body_state_msgs_WholeBodyState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          whole_body_state_msgs_WholeBodyState_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<whole_body_state_msgs::WholeBodyState,whole_body_state_msgs_msg_WholeBodyState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         whole_body_state_msgs_WholeBodyState_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<whole_body_state_msgs::WholeBodyState,whole_body_state_msgs::WholeBodyState::ConstPtr,whole_body_state_msgs_msg_WholeBodyState_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         whole_body_state_msgs_WholeBodyState_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<whole_body_state_msgs::WholeBodyState,whole_body_state_msgs_msg_WholeBodyState_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_msg_WholeBodyState_common, MATLABROSMsgInterface<whole_body_state_msgs::WholeBodyState>)
CLASS_LOADER_REGISTER_CLASS(whole_body_state_msgs_WholeBodyState_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1