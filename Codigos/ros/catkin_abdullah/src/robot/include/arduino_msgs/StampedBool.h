// Generated by gencpp from file arduino_msgs/StampedBool.msg
// DO NOT EDIT!


#ifndef ARDUINO_MSGS_MESSAGE_STAMPEDBOOL_H
#define ARDUINO_MSGS_MESSAGE_STAMPEDBOOL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace arduino_msgs
{
template <class ContainerAllocator>
struct StampedBool_
{
  typedef StampedBool_<ContainerAllocator> Type;

  StampedBool_()
    : id(0)
    , data(false)  {
    }
  StampedBool_(const ContainerAllocator& _alloc)
    : id(0)
    , data(false)  {
  (void)_alloc;
    }



   typedef int64_t _id_type;
  _id_type id;

   typedef uint8_t _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::arduino_msgs::StampedBool_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arduino_msgs::StampedBool_<ContainerAllocator> const> ConstPtr;

}; // struct StampedBool_

typedef ::arduino_msgs::StampedBool_<std::allocator<void> > StampedBool;

typedef boost::shared_ptr< ::arduino_msgs::StampedBool > StampedBoolPtr;
typedef boost::shared_ptr< ::arduino_msgs::StampedBool const> StampedBoolConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arduino_msgs::StampedBool_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arduino_msgs::StampedBool_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace arduino_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'arduino_msgs': ['/home/abdullah/catkin_abdullah/src/arduino_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::arduino_msgs::StampedBool_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arduino_msgs::StampedBool_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arduino_msgs::StampedBool_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arduino_msgs::StampedBool_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arduino_msgs::StampedBool_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arduino_msgs::StampedBool_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arduino_msgs::StampedBool_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1ea8d20576980505756a7dfb61228350";
  }

  static const char* value(const ::arduino_msgs::StampedBool_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1ea8d20576980505ULL;
  static const uint64_t static_value2 = 0x756a7dfb61228350ULL;
};

template<class ContainerAllocator>
struct DataType< ::arduino_msgs::StampedBool_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arduino_msgs/StampedBool";
  }

  static const char* value(const ::arduino_msgs::StampedBool_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arduino_msgs::StampedBool_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 id\n\
bool data\n\
";
  }

  static const char* value(const ::arduino_msgs::StampedBool_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arduino_msgs::StampedBool_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StampedBool_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arduino_msgs::StampedBool_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arduino_msgs::StampedBool_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.id);
    s << indent << "data: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDUINO_MSGS_MESSAGE_STAMPEDBOOL_H