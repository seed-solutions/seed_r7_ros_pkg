// Generated by gencpp from file seed_r7_ros_controller/HandControlRequest.msg
// DO NOT EDIT!


#ifndef SEED_R7_ROS_CONTROLLER_MESSAGE_HANDCONTROLREQUEST_H
#define SEED_R7_ROS_CONTROLLER_MESSAGE_HANDCONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace seed_r7_ros_controller
{
template <class ContainerAllocator>
struct HandControlRequest_
{
  typedef HandControlRequest_<ContainerAllocator> Type;

  HandControlRequest_()
    : position(0)
    , script()
    , current(0)  {
    }
  HandControlRequest_(const ContainerAllocator& _alloc)
    : position(0)
    , script(_alloc)
    , current(0)  {
  (void)_alloc;
    }



   typedef uint8_t _position_type;
  _position_type position;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _script_type;
  _script_type script;

   typedef uint8_t _current_type;
  _current_type current;



  enum {
    POSITION_RIGHT = 0u,
    POSITION_LEFT = 1u,
  };

  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  SCRIPT_GRASP;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  SCRIPT_RELEASE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  SCRIPT_CANCEL;

  typedef boost::shared_ptr< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct HandControlRequest_

typedef ::seed_r7_ros_controller::HandControlRequest_<std::allocator<void> > HandControlRequest;

typedef boost::shared_ptr< ::seed_r7_ros_controller::HandControlRequest > HandControlRequestPtr;
typedef boost::shared_ptr< ::seed_r7_ros_controller::HandControlRequest const> HandControlRequestConstPtr;

// constants requiring out of line definition

   

   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HandControlRequest_<ContainerAllocator>::SCRIPT_GRASP =
        
          "grasp"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HandControlRequest_<ContainerAllocator>::SCRIPT_RELEASE =
        
          "release"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HandControlRequest_<ContainerAllocator>::SCRIPT_CANCEL =
        
          "cancel"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace seed_r7_ros_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5b8557444e21ab5689c6d4cf096e0f7e";
  }

  static const char* value(const ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5b8557444e21ab56ULL;
  static const uint64_t static_value2 = 0x89c6d4cf096e0f7eULL;
};

template<class ContainerAllocator>
struct DataType< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "seed_r7_ros_controller/HandControlRequest";
  }

  static const char* value(const ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 position\n\
uint8 POSITION_RIGHT = 0\n\
uint8 POSITION_LEFT = 1\n\
\n\
string script\n\
string SCRIPT_GRASP = grasp\n\
string SCRIPT_RELEASE = release\n\
string SCRIPT_CANCEL = cancel\n\
\n\
uint8  current\n\
";
  }

  static const char* value(const ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.script);
      stream.next(m.current);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HandControlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::seed_r7_ros_controller::HandControlRequest_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.position);
    s << indent << "script: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.script);
    s << indent << "current: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.current);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEED_R7_ROS_CONTROLLER_MESSAGE_HANDCONTROLREQUEST_H