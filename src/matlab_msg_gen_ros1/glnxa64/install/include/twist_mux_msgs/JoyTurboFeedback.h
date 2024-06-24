// Generated by gencpp from file twist_mux_msgs/JoyTurboFeedback.msg
// DO NOT EDIT!


#ifndef TWIST_MUX_MSGS_MESSAGE_JOYTURBOFEEDBACK_H
#define TWIST_MUX_MSGS_MESSAGE_JOYTURBOFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace twist_mux_msgs
{
template <class ContainerAllocator>
struct JoyTurboFeedback_
{
  typedef JoyTurboFeedback_<ContainerAllocator> Type;

  JoyTurboFeedback_()
    {
    }
  JoyTurboFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct JoyTurboFeedback_

typedef ::twist_mux_msgs::JoyTurboFeedback_<std::allocator<void> > JoyTurboFeedback;

typedef boost::shared_ptr< ::twist_mux_msgs::JoyTurboFeedback > JoyTurboFeedbackPtr;
typedef boost::shared_ptr< ::twist_mux_msgs::JoyTurboFeedback const> JoyTurboFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace twist_mux_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'twist_mux_msgs': ['/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/devel/share/twist_mux_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "twist_mux_msgs/JoyTurboFeedback";
  }

  static const char* value(const ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JoyTurboFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::twist_mux_msgs::JoyTurboFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // TWIST_MUX_MSGS_MESSAGE_JOYTURBOFEEDBACK_H