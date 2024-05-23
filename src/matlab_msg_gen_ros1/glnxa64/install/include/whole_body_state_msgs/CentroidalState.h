// Generated by gencpp from file whole_body_state_msgs/CentroidalState.msg
// DO NOT EDIT!


#ifndef WHOLE_BODY_STATE_MSGS_MESSAGE_CENTROIDALSTATE_H
#define WHOLE_BODY_STATE_MSGS_MESSAGE_CENTROIDALSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>

namespace whole_body_state_msgs
{
template <class ContainerAllocator>
struct CentroidalState_
{
  typedef CentroidalState_<ContainerAllocator> Type;

  CentroidalState_()
    : com_position()
    , com_velocity()
    , base_orientation()
    , base_angular_velocity()
    , momenta()
    , momenta_rate()  {
    }
  CentroidalState_(const ContainerAllocator& _alloc)
    : com_position(_alloc)
    , com_velocity(_alloc)
    , base_orientation(_alloc)
    , base_angular_velocity(_alloc)
    , momenta(_alloc)
    , momenta_rate(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _com_position_type;
  _com_position_type com_position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _com_velocity_type;
  _com_velocity_type com_velocity;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _base_orientation_type;
  _base_orientation_type base_orientation;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _base_angular_velocity_type;
  _base_angular_velocity_type base_angular_velocity;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _momenta_type;
  _momenta_type momenta;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _momenta_rate_type;
  _momenta_rate_type momenta_rate;





  typedef boost::shared_ptr< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> const> ConstPtr;

}; // struct CentroidalState_

typedef ::whole_body_state_msgs::CentroidalState_<std::allocator<void> > CentroidalState;

typedef boost::shared_ptr< ::whole_body_state_msgs::CentroidalState > CentroidalStatePtr;
typedef boost::shared_ptr< ::whole_body_state_msgs::CentroidalState const> CentroidalStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace whole_body_state_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'whole_body_state_msgs': ['/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/src/whole_body_state_msgs/msg'], 'std_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "23ae41306b44b6e3e1e14f56a5849ac7";
  }

  static const char* value(const ::whole_body_state_msgs::CentroidalState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x23ae41306b44b6e3ULL;
  static const uint64_t static_value2 = 0xe1e14f56a5849ac7ULL;
};

template<class ContainerAllocator>
struct DataType< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "whole_body_state_msgs/CentroidalState";
  }

  static const char* value(const ::whole_body_state_msgs::CentroidalState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message describes the states of a centroidal state.\n"
"#\n"
"# The centroidal state is defined by:\n"
"#  * the position, velocity and acceleration of the CoM,\n"
"#  * the base orientation and angular velocity, and\n"
"#  * the linear and angular momentum and their rates.\n"
"# where each quantity is expressed in the world frame.\n"
"\n"
"# Center of mass \n"
"geometry_msgs/Vector3 com_position\n"
"geometry_msgs/Vector3 com_velocity\n"
"\n"
"# Base orientation\n"
"geometry_msgs/Quaternion base_orientation\n"
"geometry_msgs/Vector3 base_angular_velocity\n"
"\n"
"# Linear and angular momentum\n"
"geometry_msgs/Twist momenta\n"
"geometry_msgs/Twist momenta_rate\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
;
  }

  static const char* value(const ::whole_body_state_msgs::CentroidalState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.com_position);
      stream.next(m.com_velocity);
      stream.next(m.base_orientation);
      stream.next(m.base_angular_velocity);
      stream.next(m.momenta);
      stream.next(m.momenta_rate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CentroidalState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::whole_body_state_msgs::CentroidalState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::whole_body_state_msgs::CentroidalState_<ContainerAllocator>& v)
  {
    s << indent << "com_position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.com_position);
    s << indent << "com_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.com_velocity);
    s << indent << "base_orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.base_orientation);
    s << indent << "base_angular_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.base_angular_velocity);
    s << indent << "momenta: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.momenta);
    s << indent << "momenta_rate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.momenta_rate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WHOLE_BODY_STATE_MSGS_MESSAGE_CENTROIDALSTATE_H
