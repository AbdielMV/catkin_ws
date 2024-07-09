// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for tf_lookup/lookupTransformRequest
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
#include "tf_lookup/lookupTransform.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class TF_LOOKUP_EXPORT tf_lookup_msg_lookupTransformRequest_common : public MATLABROSMsgInterface<tf_lookup::lookupTransform::Request> {
  public:
    virtual ~tf_lookup_msg_lookupTransformRequest_common(){}
    virtual void copy_from_struct(tf_lookup::lookupTransform::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::lookupTransform::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void tf_lookup_msg_lookupTransformRequest_common::copy_from_struct(tf_lookup::lookupTransform::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //target_frame
        const matlab::data::CharArray target_frame_arr = arr["TargetFrame"];
        msg->target_frame = target_frame_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TargetFrame' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TargetFrame' is wrong type; expected a string.");
    }
    try {
        //source_frame
        const matlab::data::CharArray source_frame_arr = arr["SourceFrame"];
        msg->source_frame = source_frame_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SourceFrame' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SourceFrame' is wrong type; expected a string.");
    }
    try {
        //transform_time
        const matlab::data::StructArray transform_time_arr = arr["TransformTime"];
        auto msgClassPtr_transform_time = getCommonObject<ros::Time>("ros_msg_Time_common",loader);
        msgClassPtr_transform_time->copy_from_struct(&msg->transform_time,transform_time_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TransformTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TransformTime' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_lookupTransformRequest_common::get_arr(MDFactory_T& factory, const tf_lookup::lookupTransform::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","TargetFrame","SourceFrame","TransformTime"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/lookupTransformRequest");
    // target_frame
    auto currentElement_target_frame = (msg + ctr)->target_frame;
    outArray[ctr]["TargetFrame"] = factory.createCharArray(currentElement_target_frame);
    // source_frame
    auto currentElement_source_frame = (msg + ctr)->source_frame;
    outArray[ctr]["SourceFrame"] = factory.createCharArray(currentElement_source_frame);
    // transform_time
    auto currentElement_transform_time = (msg + ctr)->transform_time;
    auto msgClassPtr_transform_time = getCommonObject<ros::Time>("ros_msg_Time_common",loader);
    outArray[ctr]["TransformTime"] = msgClassPtr_transform_time->get_arr(factory, &currentElement_transform_time, loader);
    }
    return std::move(outArray);
  }
class TF_LOOKUP_EXPORT tf_lookup_msg_lookupTransformResponse_common : public MATLABROSMsgInterface<tf_lookup::lookupTransform::Response> {
  public:
    virtual ~tf_lookup_msg_lookupTransformResponse_common(){}
    virtual void copy_from_struct(tf_lookup::lookupTransform::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::lookupTransform::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void tf_lookup_msg_lookupTransformResponse_common::copy_from_struct(tf_lookup::lookupTransform::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //transform
        const matlab::data::StructArray transform_arr = arr["Transform"];
        auto msgClassPtr_transform = getCommonObject<geometry_msgs::TransformStamped>("geometry_msgs_msg_TransformStamped_common",loader);
        msgClassPtr_transform->copy_from_struct(&msg->transform,transform_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Transform' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Transform' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_lookupTransformResponse_common::get_arr(MDFactory_T& factory, const tf_lookup::lookupTransform::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Transform"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/lookupTransformResponse");
    // transform
    auto currentElement_transform = (msg + ctr)->transform;
    auto msgClassPtr_transform = getCommonObject<geometry_msgs::TransformStamped>("geometry_msgs_msg_TransformStamped_common",loader);
    outArray[ctr]["Transform"] = msgClassPtr_transform->get_arr(factory, &currentElement_transform, loader);
    }
    return std::move(outArray);
  } 
class TF_LOOKUP_EXPORT tf_lookup_lookupTransform_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~tf_lookup_lookupTransform_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          tf_lookup_lookupTransform_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<tf_lookup::lookupTransform::Request,tf_lookup_msg_lookupTransformRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<tf_lookup::lookupTransform::Response,tf_lookup_msg_lookupTransformResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          tf_lookup_lookupTransform_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<tf_lookup::lookupTransform::Request,tf_lookup::lookupTransform::Request::ConstPtr,tf_lookup_msg_lookupTransformRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<tf_lookup::lookupTransform::Response,tf_lookup::lookupTransform::Response::ConstPtr,tf_lookup_msg_lookupTransformResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          tf_lookup_lookupTransform_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<tf_lookup::lookupTransform::Request,tf_lookup::lookupTransform::Response,tf_lookup_msg_lookupTransformRequest_common,tf_lookup_msg_lookupTransformResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          tf_lookup_lookupTransform_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<tf_lookup::lookupTransform,tf_lookup::lookupTransform::Request,tf_lookup::lookupTransform::Response,tf_lookup_msg_lookupTransformRequest_common,tf_lookup_msg_lookupTransformResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          tf_lookup_lookupTransform_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<tf_lookup::lookupTransform::Request,tf_lookup_msg_lookupTransformRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<tf_lookup::lookupTransform::Response,tf_lookup_msg_lookupTransformResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_lookupTransformRequest_common, MATLABROSMsgInterface<tf_lookup::lookupTransform::Request>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_lookupTransformResponse_common, MATLABROSMsgInterface<tf_lookup::lookupTransform::Response>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_lookupTransform_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
