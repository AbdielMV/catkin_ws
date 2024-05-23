// Copyright 2020-2021 The MathWorks, Inc.
// Common copy functions for tf_lookup/TfStreamGoal
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
#include "tf_lookup/TfStreamAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSActionTemplates.hpp"

class TF_LOOKUP_EXPORT tf_lookup_msg_TfStreamGoal_common : public MATLABROSMsgInterface<tf_lookup::TfStreamGoal> {
  public:
    virtual ~tf_lookup_msg_TfStreamGoal_common(){}
    virtual void copy_from_struct(tf_lookup::TfStreamGoal* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::TfStreamGoal* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void tf_lookup_msg_TfStreamGoal_common::copy_from_struct(tf_lookup::TfStreamGoal* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
    try {
        //transforms
        const matlab::data::StructArray transforms_arr = arr["Transforms"];
        for (auto _transformsarr : transforms_arr) {
        	tf_lookup::Subscription _val;
        auto msgClassPtr_transforms = getCommonObject<tf_lookup::Subscription>("tf_lookup_msg_Subscription_common",loader);
        msgClassPtr_transforms->copy_from_struct(&_val,_transformsarr,loader);
        	msg->transforms.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Transforms' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Transforms' is wrong type; expected a struct.");
    }
    try {
        //subscription_id
        const matlab::data::CharArray subscription_id_arr = arr["SubscriptionId"];
        msg->subscription_id = subscription_id_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SubscriptionId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SubscriptionId' is wrong type; expected a string.");
    }
    try {
        //update
        const matlab::data::TypedArray<bool> update_arr = arr["Update"];
        msg->update = update_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Update' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Update' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_TfStreamGoal_common::get_arr(MDFactory_T& factory, const tf_lookup::TfStreamGoal* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Transforms","SubscriptionId","Update"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/TfStreamGoal");
    // transforms
    auto currentElement_transforms = (msg + ctr)->transforms;
    auto msgClassPtr_transforms = getCommonObject<tf_lookup::Subscription>("tf_lookup_msg_Subscription_common",loader);
    outArray[ctr]["Transforms"] = msgClassPtr_transforms->get_arr(factory,&currentElement_transforms[0],loader,currentElement_transforms.size());
    // subscription_id
    auto currentElement_subscription_id = (msg + ctr)->subscription_id;
    outArray[ctr]["SubscriptionId"] = factory.createCharArray(currentElement_subscription_id);
    // update
    auto currentElement_update = (msg + ctr)->update;
    outArray[ctr]["Update"] = factory.createScalar(static_cast<bool>(currentElement_update));
    }
    return std::move(outArray);
  }

class TF_LOOKUP_EXPORT tf_lookup_msg_TfStreamResult_common : public MATLABROSMsgInterface<tf_lookup::TfStreamResult> {
  public:
    virtual ~tf_lookup_msg_TfStreamResult_common(){}
    virtual void copy_from_struct(tf_lookup::TfStreamResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::TfStreamResult* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void tf_lookup_msg_TfStreamResult_common::copy_from_struct(tf_lookup::TfStreamResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
    try {
        //subscription_id
        const matlab::data::CharArray subscription_id_arr = arr["SubscriptionId"];
        msg->subscription_id = subscription_id_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SubscriptionId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SubscriptionId' is wrong type; expected a string.");
    }
    try {
        //topic
        const matlab::data::CharArray topic_arr = arr["Topic"];
        msg->topic = topic_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Topic' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Topic' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_TfStreamResult_common::get_arr(MDFactory_T& factory, const tf_lookup::TfStreamResult* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","SubscriptionId","Topic"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/TfStreamResult");
    // subscription_id
    auto currentElement_subscription_id = (msg + ctr)->subscription_id;
    outArray[ctr]["SubscriptionId"] = factory.createCharArray(currentElement_subscription_id);
    // topic
    auto currentElement_topic = (msg + ctr)->topic;
    outArray[ctr]["Topic"] = factory.createCharArray(currentElement_topic);
    }
    return std::move(outArray);
  }

class TF_LOOKUP_EXPORT tf_lookup_msg_TfStreamFeedback_common : public MATLABROSMsgInterface<tf_lookup::TfStreamFeedback> {
  public:
    virtual ~tf_lookup_msg_TfStreamFeedback_common(){}
    virtual void copy_from_struct(tf_lookup::TfStreamFeedback* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const tf_lookup::TfStreamFeedback* msg, MultiLibLoader loader, size_t size = 1);
};
  //----------------------------------------------------------------------------
  void tf_lookup_msg_TfStreamFeedback_common::copy_from_struct(tf_lookup::TfStreamFeedback* msg, const matlab::data::Struct& arr, MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T tf_lookup_msg_TfStreamFeedback_common::get_arr(MDFactory_T& factory, const tf_lookup::TfStreamFeedback* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("tf_lookup/TfStreamFeedback");
    }
    return std::move(outArray);
  }

class TF_LOOKUP_EXPORT tf_lookup_TfStream_action : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~tf_lookup_TfStream_action(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABActClientInterface> generateActClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          tf_lookup_TfStream_action::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSPublisherImpl<tf_lookup::TfStreamGoal,tf_lookup_msg_TfStreamGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSPublisherImpl<tf_lookup::TfStreamFeedback,tf_lookup_msg_TfStreamFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSPublisherImpl<tf_lookup::TfStreamResult,tf_lookup_msg_TfStreamResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         tf_lookup_TfStream_action::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSSubscriberImpl<tf_lookup::TfStreamGoal,tf_lookup::TfStreamGoal::ConstPtr,tf_lookup_msg_TfStreamGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSSubscriberImpl<tf_lookup::TfStreamFeedback,tf_lookup::TfStreamFeedback::ConstPtr,tf_lookup_msg_TfStreamFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSSubscriberImpl<tf_lookup::TfStreamResult,tf_lookup::TfStreamResult::ConstPtr,tf_lookup_msg_TfStreamResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABActClientInterface> 
          tf_lookup_TfStream_action::generateActClientInterface(){
      return std::make_shared<ROSActClientImpl<tf_lookup::TfStreamAction,tf_lookup::TfStreamGoal,tf_lookup::TfStreamFeedbackConstPtr,tf_lookup::TfStreamResultConstPtr,tf_lookup_msg_TfStreamGoal_common,tf_lookup_msg_TfStreamFeedback_common,tf_lookup_msg_TfStreamResult_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          tf_lookup_TfStream_action::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eGoal){
        ptr = std::make_shared<ROSBagWriterImpl<tf_lookup::TfStreamGoal,tf_lookup_msg_TfStreamGoal_common>>();
    }else if(type == eFeedback){
        ptr = std::make_shared<ROSBagWriterImpl<tf_lookup::TfStreamFeedback,tf_lookup_msg_TfStreamFeedback_common>>();
    }else if(type == eResult){
        ptr = std::make_shared<ROSBagWriterImpl<tf_lookup::TfStreamResult,tf_lookup_msg_TfStreamResult_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Goal' or 'Feedback' or 'Result'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_TfStreamGoal_common, MATLABROSMsgInterface<tf_lookup::TfStreamGoal>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_TfStreamFeedback_common, MATLABROSMsgInterface<tf_lookup::TfStreamFeedback>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_msg_TfStreamResult_common, MATLABROSMsgInterface<tf_lookup::TfStreamResult>)
CLASS_LOADER_REGISTER_CLASS(tf_lookup_TfStream_action, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1