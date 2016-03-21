// Generated by gencpp from file robotclient/MoveRobotResponse.msg
// DO NOT EDIT!


#ifndef ROBOTCLIENT_MESSAGE_MOVEROBOTRESPONSE_H
#define ROBOTCLIENT_MESSAGE_MOVEROBOTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotclient
{
template <class ContainerAllocator>
struct MoveRobotResponse_
{
  typedef MoveRobotResponse_<ContainerAllocator> Type;

  MoveRobotResponse_()
    : ack(0)  {
    }
  MoveRobotResponse_(const ContainerAllocator& _alloc)
    : ack(0)  {
    }



   typedef int8_t _ack_type;
  _ack_type ack;




  typedef boost::shared_ptr< ::robotclient::MoveRobotResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotclient::MoveRobotResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoveRobotResponse_

typedef ::robotclient::MoveRobotResponse_<std::allocator<void> > MoveRobotResponse;

typedef boost::shared_ptr< ::robotclient::MoveRobotResponse > MoveRobotResponsePtr;
typedef boost::shared_ptr< ::robotclient::MoveRobotResponse const> MoveRobotResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotclient::MoveRobotResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotclient::MoveRobotResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotclient

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'robotclient': ['/home/multipos2/SSYX02-16-27/robot_ws/src/robotclient/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotclient::MoveRobotResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotclient::MoveRobotResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotclient::MoveRobotResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb706f4edb700568d7fd69c87cdd4a79";
  }

  static const char* value(const ::robotclient::MoveRobotResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb706f4edb700568ULL;
  static const uint64_t static_value2 = 0xd7fd69c87cdd4a79ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotclient/MoveRobotResponse";
  }

  static const char* value(const ::robotclient::MoveRobotResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 ack\n\
\n\
";
  }

  static const char* value(const ::robotclient::MoveRobotResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ack);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MoveRobotResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotclient::MoveRobotResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotclient::MoveRobotResponse_<ContainerAllocator>& v)
  {
    s << indent << "ack: ";
    Printer<int8_t>::stream(s, indent + "  ", v.ack);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTCLIENT_MESSAGE_MOVEROBOTRESPONSE_H