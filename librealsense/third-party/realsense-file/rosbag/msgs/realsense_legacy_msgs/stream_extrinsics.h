// Generated by gencpp from file realsense_legacy_msgs/stream_extrinsics.msg
// DO NOT EDIT!


#ifndef realsense_legacy_msgs_MESSAGE_STREAM_EXTRINSICS_H
#define realsense_legacy_msgs_MESSAGE_STREAM_EXTRINSICS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <realsense_legacy_msgs/extrinsics.h>

namespace realsense_legacy_msgs
{
template <class ContainerAllocator>
struct stream_extrinsics_
{
  typedef stream_extrinsics_<ContainerAllocator> Type;

  stream_extrinsics_()
    : extrinsics()
    , reference_point_id(0)  {
    }
  stream_extrinsics_(const ContainerAllocator& _alloc)
    : extrinsics(_alloc)
    , reference_point_id(0)  {
  (void)_alloc;
    }



   typedef  ::realsense_legacy_msgs::extrinsics_<ContainerAllocator>  _extrinsics_type;
  _extrinsics_type extrinsics;

   typedef uint64_t _reference_point_id_type;
  _reference_point_id_type reference_point_id;




  typedef std::shared_ptr< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> const> ConstPtr;

}; // struct stream_extrinsics_

typedef ::realsense_legacy_msgs::stream_extrinsics_<std::allocator<void> > stream_extrinsics;

typedef std::shared_ptr< ::realsense_legacy_msgs::stream_extrinsics > stream_extrinsicsPtr;
typedef std::shared_ptr< ::realsense_legacy_msgs::stream_extrinsics const> stream_extrinsicsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> & v)
{
rs2rosinternal::message_operations::Printer< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace realsense_legacy_msgs

namespace rs2rosinternal
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'realsense_legacy_msgs': ['/home/administrator/ros/realsense_sdk_internal/sdk_internal/tools/realsense_legacy_msgs_generator/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "442fca173ef45a4a4e680d953efd6523";
  }

  static const char* value(const ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x442fca173ef45a4aULL;
  static const uint64_t static_value2 = 0x4e680d953efd6523ULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_legacy_msgs/stream_extrinsics";
  }

  static const char* value(const ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_legacy_msgs/extrinsics extrinsics\n\
uint64 reference_point_id\n\
\n\
================================================================================\n\
MSG: realsense_legacy_msgs/extrinsics\n\
float32[9] rotation    # column-major 3x3 rotation matrix \n\
float32[3] translation # 3 element translation vector, in meters \n\
";
  }

  static const char* value(const ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.extrinsics);
      stream.next(m.reference_point_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct stream_extrinsics_

} // namespace serialization
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_legacy_msgs::stream_extrinsics_<ContainerAllocator>& v)
  {
    s << indent << "extrinsics: ";
    s << std::endl;
    Printer< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >::stream(s, indent + "  ", v.extrinsics);
    s << indent << "reference_point_id: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.reference_point_id);
  }
};

} // namespace message_operations
} // namespace rs2rosinternal

#endif // realsense_legacy_msgs_MESSAGE_STREAM_EXTRINSICS_H
