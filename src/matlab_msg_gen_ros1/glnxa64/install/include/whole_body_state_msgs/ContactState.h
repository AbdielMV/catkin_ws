// Generated by gencpp from file whole_body_state_msgs/ContactState.msg
// DO NOT EDIT!


#ifndef WHOLE_BODY_STATE_MSGS_MESSAGE_CONTACTSTATE_H
#define WHOLE_BODY_STATE_MSGS_MESSAGE_CONTACTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

namespace whole_body_state_msgs
{
template <class ContainerAllocator>
struct ContactState_
{
  typedef ContactState_<ContainerAllocator> Type;

  ContactState_()
    : name()
    , type(0)
    , pose()
    , velocity()
    , wrench()
    , surface_normal()
    , friction_coefficient(0.0)  {
    }
  ContactState_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , type(0)
    , pose(_alloc)
    , velocity(_alloc)
    , wrench(_alloc)
    , surface_normal(_alloc)
    , friction_coefficient(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef uint8_t _type_type;
  _type_type type;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Wrench_<ContainerAllocator>  _wrench_type;
  _wrench_type wrench;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _surface_normal_type;
  _surface_normal_type surface_normal;

   typedef double _friction_coefficient_type;
  _friction_coefficient_type friction_coefficient;



  enum {
 
    locomotion = 0u,
 
    manipulation = 1u,
  };


  typedef boost::shared_ptr< ::whole_body_state_msgs::ContactState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::whole_body_state_msgs::ContactState_<ContainerAllocator> const> ConstPtr;

}; // struct ContactState_

typedef ::whole_body_state_msgs::ContactState_<std::allocator<void> > ContactState;

typedef boost::shared_ptr< ::whole_body_state_msgs::ContactState > ContactStatePtr;
typedef boost::shared_ptr< ::whole_body_state_msgs::ContactState const> ContactStateConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::whole_body_state_msgs::ContactState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace whole_body_state_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'whole_body_state_msgs': ['/home/abdiel/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/src/whole_body_state_msgs/msg'], 'std_msgs': ['/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/home/abdiel/MATLAB/R2021b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::whole_body_state_msgs::ContactState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whole_body_state_msgs::ContactState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whole_body_state_msgs::ContactState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b02d1e064d7bfebaee1f4ca22e691c9c";
  }

  static const char* value(const ::whole_body_state_msgs::ContactState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb02d1e064d7bfebaULL;
  static const uint64_t static_value2 = 0xee1f4ca22e691c9cULL;
};

template<class ContainerAllocator>
struct DataType< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "whole_body_state_msgs/ContactState";
  }

  static const char* value(const ::whole_body_state_msgs::ContactState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message describes the state of contact or end-effector body.\n"
"#\n"
"# The contact state is expressed in the world frame. A contact state is\n"
"# defined by:\n"
"#  * type of contact\n"
"#  * the frame name\n"
"#  * the pose of the contact,\n"
"#  * the velocity of the contact,\n"
"#  * the wrench of the contact\n"
"#  * the normal vector that defines the surface\n"
"#  * the friction coefficient of the surface\n"
"\n"
"# Type of contact identifiers\n"
"uint8 locomotion=0\n"
"uint8 manipulation=1\n"
"\n"
"# Name of the contact body\n"
"string name\n"
"uint8 type\n"
"\n"
"# State of the contact body\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist velocity\n"
"geometry_msgs/Wrench wrench\n"
"geometry_msgs/Vector3 surface_normal\n"
"float64 friction_coefficient\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
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
"MSG: geometry_msgs/Wrench\n"
"# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
;
  }

  static const char* value(const ::whole_body_state_msgs::ContactState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.type);
      stream.next(m.pose);
      stream.next(m.velocity);
      stream.next(m.wrench);
      stream.next(m.surface_normal);
      stream.next(m.friction_coefficient);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ContactState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::whole_body_state_msgs::ContactState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::whole_body_state_msgs::ContactState_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "wrench: ";
    s << std::endl;
    Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >::stream(s, indent + "  ", v.wrench);
    s << indent << "surface_normal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.surface_normal);
    s << indent << "friction_coefficient: ";
    Printer<double>::stream(s, indent + "  ", v.friction_coefficient);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WHOLE_BODY_STATE_MSGS_MESSAGE_CONTACTSTATE_H
