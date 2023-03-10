// Generated by gencpp from file ICET/Num.msg
// DO NOT EDIT!


#ifndef ICET_MESSAGE_NUM_H
#define ICET_MESSAGE_NUM_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ICET
{
template <class ContainerAllocator>
struct Num_
{
  typedef Num_<ContainerAllocator> Type;

  Num_()
    : timestamp()
    , restart(false)
    , frame(0)
    , status()
    , true_transform()  {
    }
  Num_(const ContainerAllocator& _alloc)
    : timestamp()
    , restart(false)
    , frame(0)
    , status(_alloc)
    , true_transform(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _restart_type;
  _restart_type restart;

   typedef int32_t _frame_type;
  _frame_type frame;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _status_type;
  _status_type status;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _true_transform_type;
  _true_transform_type true_transform;





  typedef boost::shared_ptr< ::ICET::Num_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ICET::Num_<ContainerAllocator> const> ConstPtr;

}; // struct Num_

typedef ::ICET::Num_<std::allocator<void> > Num;

typedef boost::shared_ptr< ::ICET::Num > NumPtr;
typedef boost::shared_ptr< ::ICET::Num const> NumConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ICET::Num_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ICET::Num_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ICET::Num_<ContainerAllocator1> & lhs, const ::ICET::Num_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.restart == rhs.restart &&
    lhs.frame == rhs.frame &&
    lhs.status == rhs.status &&
    lhs.true_transform == rhs.true_transform;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ICET::Num_<ContainerAllocator1> & lhs, const ::ICET::Num_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ICET

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ICET::Num_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ICET::Num_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ICET::Num_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ICET::Num_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ICET::Num_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ICET::Num_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ICET::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "93042447ed01f85739c5e6e8683f8ec7";
  }

  static const char* value(const ::ICET::Num_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x93042447ed01f857ULL;
  static const uint64_t static_value2 = 0x39c5e6e8683f8ec7ULL;
};

template<class ContainerAllocator>
struct DataType< ::ICET::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ICET/Num";
  }

  static const char* value(const ::ICET::Num_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ICET::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"bool restart\n"
"int32 frame\n"
"string status\n"
"float32[] true_transform\n"
;
  }

  static const char* value(const ::ICET::Num_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ICET::Num_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.restart);
      stream.next(m.frame);
      stream.next(m.status);
      stream.next(m.true_transform);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Num_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ICET::Num_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ICET::Num_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "restart: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.restart);
    s << indent << "frame: ";
    Printer<int32_t>::stream(s, indent + "  ", v.frame);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.status);
    s << indent << "true_transform[]" << std::endl;
    for (size_t i = 0; i < v.true_transform.size(); ++i)
    {
      s << indent << "  true_transform[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.true_transform[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ICET_MESSAGE_NUM_H