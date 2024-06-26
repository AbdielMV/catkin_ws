// Generated by gencpp from file tf_lookup/TfLookupFeedback.msg
// DO NOT EDIT!


#ifndef TF_LOOKUP_MESSAGE_TFLOOKUPFEEDBACK_H
#define TF_LOOKUP_MESSAGE_TFLOOKUPFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tf_lookup
{
template <class ContainerAllocator>
struct TfLookupFeedback_
{
  typedef TfLookupFeedback_<ContainerAllocator> Type;

  TfLookupFeedback_()
    {
    }
  TfLookupFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct TfLookupFeedback_

typedef ::tf_lookup::TfLookupFeedback_<std::allocator<void> > TfLookupFeedback;

typedef boost::shared_ptr< ::tf_lookup::TfLookupFeedback > TfLookupFeedbackPtr;
typedef boost::shared_ptr< ::tf_lookup::TfLookupFeedback const> TfLookupFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tf_lookup::TfLookupFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tf_lookup

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'tf_lookup': ['/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/src/tf_lookup/msg', '/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/devel/share/tf_lookup/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::tf_lookup::TfLookupFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf_lookup/TfLookupFeedback";
  }

  static const char* value(const ::tf_lookup::TfLookupFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the feedback\n"
"\n"
;
  }

  static const char* value(const ::tf_lookup::TfLookupFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TfLookupFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tf_lookup::TfLookupFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::tf_lookup::TfLookupFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // TF_LOOKUP_MESSAGE_TFLOOKUPFEEDBACK_H
