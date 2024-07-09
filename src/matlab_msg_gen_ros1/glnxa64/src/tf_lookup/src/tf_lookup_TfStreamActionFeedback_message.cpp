// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for tf_lookup/TfStreamActionFeedback
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
#include "tf_lookup/TfStreamActionFeedback.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class TF_LOOKUP_EXPORT tf_lookup_msg_TfStreamActionFeedback_common : public MATLABROSMsgInterface<tf_lookup::TfStreamActionFeedback> {
  public:
    virtual ~tf_lookup_msg_TfStreamActionFeedback_common(){}
    virtual void copy_from_struct(tf_lookup::TfStreamActionFeedback* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::TfStreamActionFeedback* msg, MultiLibLoader loader, size_t size = 1);
};
  void tf_lookup_msg_TfStreamActionFeedback_common::copy_from_struct(tf_lookup::TfStreamActionFeedback* msg, const matlab::data::Struct& arr,
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
        //feedback
        const matlab::data::StructArray feedback_arr = arr["Feedback"];
        auto msgClassPtr_feedback = getCommonObject<tf_lookup::TfStreamFeedback>("tf_lookup_msg_TfStreamFeedback_common",loader);
        msgClassPtr_feedback->copy_from_struct(&msg->feedback,feedback_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Feedback' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Feedback' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_TfStreamActionFeedback_common::get_arr(MDFactory_T& factory, const tf_lookup::TfStreamActionFeedback* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Status","Feedback"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/TfStreamActionFeedback");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // status
    auto currentElement_status = (msg + ctr)->status;
    auto msgClassPtr_status = getCommonObject<actionlib_msgs::GoalStatus>("actionlib_msgs_msg_GoalStatus_common",loader);
    outArray[ctr]["Status"] = msgClassPtr_status->get_arr(factory, &currentElement_status, loader);
    // feedback
    auto currentElement_feedback = (msg + ctr)->feedback;
    auto msgClassPtr_feedback = getCommonObject<tf_lookup::TfStreamFeedback>("tf_lookup_msg_TfStreamFeedback_common",loader);
    outArray[ctr]["Feedback"] = msgClassPtr_feedback->get_arr(factory, &currentElement_feedback, loader);
    }
    return std::move(outArray);
  } 
class TF_LOOKUP_EXPORT tf_lookup_TfStreamActionFeedback_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~tf_lookup_TfStreamActionFeedback_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          tf_lookup_TfStreamActionFeedback_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<tf_lookup::TfStreamActionFeedback,tf_lookup_msg_TfStreamActionFeedback_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         tf_lookup_TfStreamActionFeedback_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<tf_lookup::TfStreamActionFeedback,tf_lookup::TfStreamActionFeedback::ConstPtr,tf_lookup_msg_TfStreamActionFeedback_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         tf_lookup_TfStreamActionFeedback_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<tf_lookup::TfStreamActionFeedback,tf_lookup_msg_TfStreamActionFeedback_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_TfStreamActionFeedback_common, MATLABROSMsgInterface<tf_lookup::TfStreamActionFeedback>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_TfStreamActionFeedback_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1