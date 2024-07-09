// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for dynamic_introspection/DoubleParameter
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
#include "dynamic_introspection/DoubleParameter.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class DYNAMIC_INTROSPECTION_EXPORT dynamic_introspection_msg_DoubleParameter_common : public MATLABROSMsgInterface<dynamic_introspection::DoubleParameter> {
  public:
    virtual ~dynamic_introspection_msg_DoubleParameter_common(){}
    virtual void copy_from_struct(dynamic_introspection::DoubleParameter* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const dynamic_introspection::DoubleParameter* msg, MultiLibLoader loader, size_t size = 1);
};
  void dynamic_introspection_msg_DoubleParameter_common::copy_from_struct(dynamic_introspection::DoubleParameter* msg, const matlab::data::Struct& arr,
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
        //value
        const matlab::data::TypedArray<double> value_arr = arr["Value"];
        msg->value = value_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Value' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Value' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T dynamic_introspection_msg_DoubleParameter_common::get_arr(MDFactory_T& factory, const dynamic_introspection::DoubleParameter* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Name","Value"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("dynamic_introspection/DoubleParameter");
    // name
    auto currentElement_name = (msg + ctr)->name;
    outArray[ctr]["Name"] = factory.createCharArray(currentElement_name);
    // value
    auto currentElement_value = (msg + ctr)->value;
    outArray[ctr]["Value"] = factory.createScalar(currentElement_value);
    }
    return std::move(outArray);
  } 
class DYNAMIC_INTROSPECTION_EXPORT dynamic_introspection_DoubleParameter_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~dynamic_introspection_DoubleParameter_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          dynamic_introspection_DoubleParameter_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<dynamic_introspection::DoubleParameter,dynamic_introspection_msg_DoubleParameter_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         dynamic_introspection_DoubleParameter_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<dynamic_introspection::DoubleParameter,dynamic_introspection::DoubleParameter::ConstPtr,dynamic_introspection_msg_DoubleParameter_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         dynamic_introspection_DoubleParameter_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<dynamic_introspection::DoubleParameter,dynamic_introspection_msg_DoubleParameter_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(dynamic_introspection_msg_DoubleParameter_common, MATLABROSMsgInterface<dynamic_introspection::DoubleParameter>)
CLASS_LOADER_REGISTER_CLASS(dynamic_introspection_DoubleParameter_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1