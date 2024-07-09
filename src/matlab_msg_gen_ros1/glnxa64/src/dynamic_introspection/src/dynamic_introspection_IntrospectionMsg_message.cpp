// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for dynamic_introspection/IntrospectionMsg
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
#include "dynamic_introspection/IntrospectionMsg.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class DYNAMIC_INTROSPECTION_EXPORT dynamic_introspection_msg_IntrospectionMsg_common : public MATLABROSMsgInterface<dynamic_introspection::IntrospectionMsg> {
  public:
    virtual ~dynamic_introspection_msg_IntrospectionMsg_common(){}
    virtual void copy_from_struct(dynamic_introspection::IntrospectionMsg* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const dynamic_introspection::IntrospectionMsg* msg, MultiLibLoader loader, size_t size = 1);
};
  void dynamic_introspection_msg_IntrospectionMsg_common::copy_from_struct(dynamic_introspection::IntrospectionMsg* msg, const matlab::data::Struct& arr,
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
        //bools
        const matlab::data::StructArray bools_arr = arr["Bools"];
        for (auto _boolsarr : bools_arr) {
        	dynamic_introspection::BoolParameter _val;
        auto msgClassPtr_bools = getCommonObject<dynamic_introspection::BoolParameter>("dynamic_introspection_msg_BoolParameter_common",loader);
        msgClassPtr_bools->copy_from_struct(&_val,_boolsarr,loader);
        	msg->bools.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Bools' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Bools' is wrong type; expected a struct.");
    }
    try {
        //doubles
        const matlab::data::StructArray doubles_arr = arr["Doubles"];
        for (auto _doublesarr : doubles_arr) {
        	dynamic_introspection::DoubleParameter _val;
        auto msgClassPtr_doubles = getCommonObject<dynamic_introspection::DoubleParameter>("dynamic_introspection_msg_DoubleParameter_common",loader);
        msgClassPtr_doubles->copy_from_struct(&_val,_doublesarr,loader);
        	msg->doubles.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Doubles' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Doubles' is wrong type; expected a struct.");
    }
    try {
        //ints
        const matlab::data::StructArray ints_arr = arr["Ints"];
        for (auto _intsarr : ints_arr) {
        	dynamic_introspection::IntParameter _val;
        auto msgClassPtr_ints = getCommonObject<dynamic_introspection::IntParameter>("dynamic_introspection_msg_IntParameter_common",loader);
        msgClassPtr_ints->copy_from_struct(&_val,_intsarr,loader);
        	msg->ints.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Ints' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Ints' is wrong type; expected a struct.");
    }
    try {
        //markers
        const matlab::data::StructArray markers_arr = arr["Markers"];
        for (auto _markersarr : markers_arr) {
        	dynamic_introspection::MarkerParameter _val;
        auto msgClassPtr_markers = getCommonObject<dynamic_introspection::MarkerParameter>("dynamic_introspection_msg_MarkerParameter_common",loader);
        msgClassPtr_markers->copy_from_struct(&_val,_markersarr,loader);
        	msg->markers.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Markers' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Markers' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T dynamic_introspection_msg_IntrospectionMsg_common::get_arr(MDFactory_T& factory, const dynamic_introspection::IntrospectionMsg* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Bools","Doubles","Ints","Markers"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("dynamic_introspection/IntrospectionMsg");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // bools
    auto currentElement_bools = (msg + ctr)->bools;
    auto msgClassPtr_bools = getCommonObject<dynamic_introspection::BoolParameter>("dynamic_introspection_msg_BoolParameter_common",loader);
    outArray[ctr]["Bools"] = msgClassPtr_bools->get_arr(factory,&currentElement_bools[0],loader,currentElement_bools.size());
    // doubles
    auto currentElement_doubles = (msg + ctr)->doubles;
    auto msgClassPtr_doubles = getCommonObject<dynamic_introspection::DoubleParameter>("dynamic_introspection_msg_DoubleParameter_common",loader);
    outArray[ctr]["Doubles"] = msgClassPtr_doubles->get_arr(factory,&currentElement_doubles[0],loader,currentElement_doubles.size());
    // ints
    auto currentElement_ints = (msg + ctr)->ints;
    auto msgClassPtr_ints = getCommonObject<dynamic_introspection::IntParameter>("dynamic_introspection_msg_IntParameter_common",loader);
    outArray[ctr]["Ints"] = msgClassPtr_ints->get_arr(factory,&currentElement_ints[0],loader,currentElement_ints.size());
    // markers
    auto currentElement_markers = (msg + ctr)->markers;
    auto msgClassPtr_markers = getCommonObject<dynamic_introspection::MarkerParameter>("dynamic_introspection_msg_MarkerParameter_common",loader);
    outArray[ctr]["Markers"] = msgClassPtr_markers->get_arr(factory,&currentElement_markers[0],loader,currentElement_markers.size());
    }
    return std::move(outArray);
  } 
class DYNAMIC_INTROSPECTION_EXPORT dynamic_introspection_IntrospectionMsg_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~dynamic_introspection_IntrospectionMsg_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          dynamic_introspection_IntrospectionMsg_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<dynamic_introspection::IntrospectionMsg,dynamic_introspection_msg_IntrospectionMsg_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         dynamic_introspection_IntrospectionMsg_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<dynamic_introspection::IntrospectionMsg,dynamic_introspection::IntrospectionMsg::ConstPtr,dynamic_introspection_msg_IntrospectionMsg_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         dynamic_introspection_IntrospectionMsg_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<dynamic_introspection::IntrospectionMsg,dynamic_introspection_msg_IntrospectionMsg_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(dynamic_introspection_msg_IntrospectionMsg_common, MATLABROSMsgInterface<dynamic_introspection::IntrospectionMsg>)
CLASS_LOADER_REGISTER_CLASS(dynamic_introspection_IntrospectionMsg_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1