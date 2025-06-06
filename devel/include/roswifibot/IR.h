// Generated by gencpp from file roswifibot/IR.msg
// DO NOT EDIT!


#ifndef ROSWIFIBOT_MESSAGE_IR_H
#define ROSWIFIBOT_MESSAGE_IR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace roswifibot
{
template <class ContainerAllocator>
struct IR_
{
  typedef IR_<ContainerAllocator> Type;

  IR_()
    : IR_front_left(0.0)
    , IR_back_left(0.0)
    , IR_front_right(0.0)
    , IR_back_right(0.0)  {
    }
  IR_(const ContainerAllocator& _alloc)
    : IR_front_left(0.0)
    , IR_back_left(0.0)
    , IR_front_right(0.0)
    , IR_back_right(0.0)  {
  (void)_alloc;
    }



   typedef double _IR_front_left_type;
  _IR_front_left_type IR_front_left;

   typedef double _IR_back_left_type;
  _IR_back_left_type IR_back_left;

   typedef double _IR_front_right_type;
  _IR_front_right_type IR_front_right;

   typedef double _IR_back_right_type;
  _IR_back_right_type IR_back_right;





  typedef boost::shared_ptr< ::roswifibot::IR_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roswifibot::IR_<ContainerAllocator> const> ConstPtr;

}; // struct IR_

typedef ::roswifibot::IR_<std::allocator<void> > IR;

typedef boost::shared_ptr< ::roswifibot::IR > IRPtr;
typedef boost::shared_ptr< ::roswifibot::IR const> IRConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roswifibot::IR_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roswifibot::IR_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roswifibot::IR_<ContainerAllocator1> & lhs, const ::roswifibot::IR_<ContainerAllocator2> & rhs)
{
  return lhs.IR_front_left == rhs.IR_front_left &&
    lhs.IR_back_left == rhs.IR_back_left &&
    lhs.IR_front_right == rhs.IR_front_right &&
    lhs.IR_back_right == rhs.IR_back_right;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roswifibot::IR_<ContainerAllocator1> & lhs, const ::roswifibot::IR_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roswifibot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::roswifibot::IR_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roswifibot::IR_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roswifibot::IR_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roswifibot::IR_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roswifibot::IR_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roswifibot::IR_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roswifibot::IR_<ContainerAllocator> >
{
  static const char* value()
  {
    return "08989c603acc510242caf5149106a2a8";
  }

  static const char* value(const ::roswifibot::IR_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x08989c603acc5102ULL;
  static const uint64_t static_value2 = 0x42caf5149106a2a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::roswifibot::IR_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roswifibot/IR";
  }

  static const char* value(const ::roswifibot::IR_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roswifibot::IR_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 IR_front_left\n"
"float64 IR_back_left\n"
"float64 IR_front_right\n"
"float64 IR_back_right\n"
;
  }

  static const char* value(const ::roswifibot::IR_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roswifibot::IR_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.IR_front_left);
      stream.next(m.IR_back_left);
      stream.next(m.IR_front_right);
      stream.next(m.IR_back_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IR_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roswifibot::IR_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roswifibot::IR_<ContainerAllocator>& v)
  {
    s << indent << "IR_front_left: ";
    Printer<double>::stream(s, indent + "  ", v.IR_front_left);
    s << indent << "IR_back_left: ";
    Printer<double>::stream(s, indent + "  ", v.IR_back_left);
    s << indent << "IR_front_right: ";
    Printer<double>::stream(s, indent + "  ", v.IR_front_right);
    s << indent << "IR_back_right: ";
    Printer<double>::stream(s, indent + "  ", v.IR_back_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSWIFIBOT_MESSAGE_IR_H
