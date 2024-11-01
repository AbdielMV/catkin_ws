// Generated by gencpp from file tf_lookup/TfStreamGoal.msg
// DO NOT EDIT!


#ifndef TF_LOOKUP_MESSAGE_TFSTREAMGOAL_H
#define TF_LOOKUP_MESSAGE_TFSTREAMGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tf_lookup/Subscription.h>

namespace tf_lookup
{
template <class ContainerAllocator>
struct TfStreamGoal_
{
  typedef TfStreamGoal_<ContainerAllocator> Type;

  TfStreamGoal_()
    : transforms()
    , subscription_id()
    , update(false)  {
    }
  TfStreamGoal_(const ContainerAllocator& _alloc)
    : transforms(_alloc)
    , subscription_id(_alloc)
    , update(false)  {
  (void)_alloc;
    }



   typedef std::vector< ::tf_lookup::Subscription_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::tf_lookup::Subscription_<ContainerAllocator> >::other >  _transforms_type;
  _transforms_type transforms;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _subscription_id_type;
  _subscription_id_type subscription_id;

   typedef uint8_t _update_type;
  _update_type update;





  typedef boost::shared_ptr< ::tf_lookup::TfStreamGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tf_lookup::TfStreamGoal_<ContainerAllocator> const> ConstPtr;

}; // struct TfStreamGoal_

typedef ::tf_lookup::TfStreamGoal_<std::allocator<void> > TfStreamGoal;

typedef boost::shared_ptr< ::tf_lookup::TfStreamGoal > TfStreamGoalPtr;
typedef boost::shared_ptr< ::tf_lookup::TfStreamGoal const> TfStreamGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tf_lookup::TfStreamGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tf_lookup

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'tf_lookup': ['/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/src/tf_lookup/msg', '/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/devel/share/tf_lookup/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfStreamGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfStreamGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfStreamGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e00b5ec9adf5765d948ec802ab721a4a";
  }

  static const char* value(const ::tf_lookup::TfStreamGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe00b5ec9adf5765dULL;
  static const uint64_t static_value2 = 0x948ec802ab721a4aULL;
};

template<class ContainerAllocator>
struct DataType< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf_lookup/TfStreamGoal";
  }

  static const char* value(const ::tf_lookup::TfStreamGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the goal\n"
"Subscription[] transforms\n"
"string subscription_id\n"
"bool update\n"
"\n"
"================================================================================\n"
"MSG: tf_lookup/Subscription\n"
"string target\n"
"string source\n"
;
  }

  static const char* value(const ::tf_lookup::TfStreamGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transforms);
      stream.next(m.subscription_id);
      stream.next(m.update);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TfStreamGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tf_lookup::TfStreamGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tf_lookup::TfStreamGoal_<ContainerAllocator>& v)
  {
    s << indent << "transforms[]" << std::endl;
    for (size_t i = 0; i < v.transforms.size(); ++i)
    {
      s << indent << "  transforms[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::tf_lookup::Subscription_<ContainerAllocator> >::stream(s, indent + "    ", v.transforms[i]);
    }
    s << indent << "subscription_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.subscription_id);
    s << indent << "update: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.update);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TF_LOOKUP_MESSAGE_TFSTREAMGOAL_H
