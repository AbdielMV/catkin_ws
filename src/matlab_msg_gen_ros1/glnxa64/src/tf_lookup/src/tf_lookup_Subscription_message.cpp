// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for tf_lookup/Subscription
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
#include "tf_lookup/Subscription.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class TF_LOOKUP_EXPORT tf_lookup_msg_Subscription_common : public MATLABROSMsgInterface<tf_lookup::Subscription> {
  public:
    virtual ~tf_lookup_msg_Subscription_common(){}
    virtual void copy_from_struct(tf_lookup::Subscription* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::Subscription* msg, MultiLibLoader loader, size_t size = 1);
};
  void tf_lookup_msg_Subscription_common::copy_from_struct(tf_lookup::Subscription* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //target
        const matlab::data::CharArray target_arr = arr["Target"];
        msg->target = target_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Target' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Target' is wrong type; expected a string.");
    }
    try {
        //source
        const matlab::data::CharArray source_arr = arr["Source"];
        msg->source = source_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Source' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Source' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_Subscription_common::get_arr(MDFactory_T& factory, const tf_lookup::Subscription* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Target","Source"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/Subscription");
    // target
    auto currentElement_target = (msg + ctr)->target;
    outArray[ctr]["Target"] = factory.createCharArray(currentElement_target);
    // source
    auto currentElement_source = (msg + ctr)->source;
    outArray[ctr]["Source"] = factory.createCharArray(currentElement_source);
    }
    return std::move(outArray);
  } 
class TF_LOOKUP_EXPORT tf_lookup_Subscription_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~tf_lookup_Subscription_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          tf_lookup_Subscription_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<tf_lookup::Subscription,tf_lookup_msg_Subscription_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         tf_lookup_Subscription_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<tf_lookup::Subscription,tf_lookup::Subscription::ConstPtr,tf_lookup_msg_Subscription_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         tf_lookup_Subscription_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<tf_lookup::Subscription,tf_lookup_msg_Subscription_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_Subscription_common, MATLABROSMsgInterface<tf_lookup::Subscription>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_Subscription_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1