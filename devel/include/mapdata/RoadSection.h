// Generated by gencpp from file mapdata/RoadSection.msg
// DO NOT EDIT!


#ifndef MAPDATA_MESSAGE_ROADSECTION_H
#define MAPDATA_MESSAGE_ROADSECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mapdata/Position.h>
#include <mapdata/Position.h>

namespace mapdata
{
template <class ContainerAllocator>
struct RoadSection_
{
  typedef RoadSection_<ContainerAllocator> Type;

  RoadSection_()
    : left()
    , right()
    , length(0)
    , stopline_offset(0)
    , priority_sign(0)
    , name()  {
    }
  RoadSection_(const ContainerAllocator& _alloc)
    : left(_alloc)
    , right(_alloc)
    , length(0)
    , stopline_offset(0)
    , priority_sign(0)
    , name(_alloc)  {
  (void)_alloc;
    }



   typedef  ::mapdata::Position_<ContainerAllocator>  _left_type;
  _left_type left;

   typedef  ::mapdata::Position_<ContainerAllocator>  _right_type;
  _right_type right;

   typedef int32_t _length_type;
  _length_type length;

   typedef int32_t _stopline_offset_type;
  _stopline_offset_type stopline_offset;

   typedef uint8_t _priority_sign_type;
  _priority_sign_type priority_sign;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(PRIORITY_ROAD)
  #undef PRIORITY_ROAD
#endif
#if defined(_WIN32) && defined(GIVE_WAY)
  #undef GIVE_WAY
#endif
#if defined(_WIN32) && defined(STOP_SIGN)
  #undef STOP_SIGN
#endif
#if defined(_WIN32) && defined(TRAFFIC_LIGHT)
  #undef TRAFFIC_LIGHT
#endif
#if defined(_WIN32) && defined(BOOKING)
  #undef BOOKING
#endif

  enum {
    PRIORITY_ROAD = 0u,
    GIVE_WAY = 1u,
    STOP_SIGN = 2u,
    TRAFFIC_LIGHT = 3u,
    BOOKING = 4u,
  };


  typedef boost::shared_ptr< ::mapdata::RoadSection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mapdata::RoadSection_<ContainerAllocator> const> ConstPtr;

}; // struct RoadSection_

typedef ::mapdata::RoadSection_<std::allocator<void> > RoadSection;

typedef boost::shared_ptr< ::mapdata::RoadSection > RoadSectionPtr;
typedef boost::shared_ptr< ::mapdata::RoadSection const> RoadSectionConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mapdata::RoadSection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mapdata::RoadSection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mapdata::RoadSection_<ContainerAllocator1> & lhs, const ::mapdata::RoadSection_<ContainerAllocator2> & rhs)
{
  return lhs.left == rhs.left &&
    lhs.right == rhs.right &&
    lhs.length == rhs.length &&
    lhs.stopline_offset == rhs.stopline_offset &&
    lhs.priority_sign == rhs.priority_sign &&
    lhs.name == rhs.name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mapdata::RoadSection_<ContainerAllocator1> & lhs, const ::mapdata::RoadSection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mapdata

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mapdata::RoadSection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mapdata::RoadSection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mapdata::RoadSection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mapdata::RoadSection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mapdata::RoadSection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mapdata::RoadSection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mapdata::RoadSection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e564107726b89210b4492752f37581d9";
  }

  static const char* value(const ::mapdata::RoadSection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe564107726b89210ULL;
  static const uint64_t static_value2 = 0xb4492752f37581d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::mapdata::RoadSection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mapdata/RoadSection";
  }

  static const char* value(const ::mapdata::RoadSection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mapdata::RoadSection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# If you're standing on the road looking towards the intersection, the left\n"
"# position is the leftmost edge of the rectangle closest to the intersection.\n"
"#    |       |\n"
"#    |       |\n"
"#  right    left\n"
"# ---+       +----\n"
"#\n"
"Position left\n"
"Position right\n"
"\n"
"# How far the road section extends from the intersection\n"
"int32 length\n"
"\n"
"# How far from the intersection the cars should stop\n"
"int32 stopline_offset\n"
"\n"
"# Enumeration (just constants) of priority signs\n"
"uint8 PRIORITY_ROAD=0\n"
"uint8 GIVE_WAY=1\n"
"uint8 STOP_SIGN=2\n"
"uint8 TRAFFIC_LIGHT=3\n"
"uint8 BOOKING=4\n"
"\n"
"uint8 priority_sign\n"
"\n"
"# A bit redundant but nice for pretty printing\n"
"string name\n"
"\n"
"================================================================================\n"
"MSG: mapdata/Position\n"
"int32 x\n"
"int32 y\n"
;
  }

  static const char* value(const ::mapdata::RoadSection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mapdata::RoadSection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
      stream.next(m.length);
      stream.next(m.stopline_offset);
      stream.next(m.priority_sign);
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RoadSection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mapdata::RoadSection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mapdata::RoadSection_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    s << std::endl;
    Printer< ::mapdata::Position_<ContainerAllocator> >::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    s << std::endl;
    Printer< ::mapdata::Position_<ContainerAllocator> >::stream(s, indent + "  ", v.right);
    s << indent << "length: ";
    Printer<int32_t>::stream(s, indent + "  ", v.length);
    s << indent << "stopline_offset: ";
    Printer<int32_t>::stream(s, indent + "  ", v.stopline_offset);
    s << indent << "priority_sign: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.priority_sign);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAPDATA_MESSAGE_ROADSECTION_H
