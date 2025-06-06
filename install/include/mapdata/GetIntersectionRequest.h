// Generated by gencpp from file mapdata/GetIntersectionRequest.msg
// DO NOT EDIT!


#ifndef MAPDATA_MESSAGE_GETINTERSECTIONREQUEST_H
#define MAPDATA_MESSAGE_GETINTERSECTIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mapdata
{
template <class ContainerAllocator>
struct GetIntersectionRequest_
{
  typedef GetIntersectionRequest_<ContainerAllocator> Type;

  GetIntersectionRequest_()
    {
    }
  GetIntersectionRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::mapdata::GetIntersectionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mapdata::GetIntersectionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetIntersectionRequest_

typedef ::mapdata::GetIntersectionRequest_<std::allocator<void> > GetIntersectionRequest;

typedef boost::shared_ptr< ::mapdata::GetIntersectionRequest > GetIntersectionRequestPtr;
typedef boost::shared_ptr< ::mapdata::GetIntersectionRequest const> GetIntersectionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mapdata::GetIntersectionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace mapdata

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mapdata::GetIntersectionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mapdata::GetIntersectionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mapdata::GetIntersectionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::mapdata::GetIntersectionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mapdata/GetIntersectionRequest";
  }

  static const char* value(const ::mapdata::GetIntersectionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::mapdata::GetIntersectionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetIntersectionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mapdata::GetIntersectionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::mapdata::GetIntersectionRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MAPDATA_MESSAGE_GETINTERSECTIONREQUEST_H
