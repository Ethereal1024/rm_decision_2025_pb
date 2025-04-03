// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from iw_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice
#include "iw_interfaces/msg/detail/robot__rosidl_typesupport_fastrtps_cpp.hpp"
#include "iw_interfaces/msg/detail/robot__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::PoseStamped &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::PoseStamped &);
size_t get_serialized_size(
  const geometry_msgs::msg::PoseStamped &,
  size_t current_alignment);
size_t
max_serialized_size_PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace iw_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_iw_interfaces
cdr_serialize(
  const iw_interfaces::msg::Robot & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: label
  cdr << ros_message.label;
  // Member: faction
  cdr << ros_message.faction;
  // Member: id
  cdr << ros_message.id;
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  // Member: hp
  cdr << ros_message.hp;
  // Member: level
  cdr << ros_message.level;
  // Member: attack
  cdr << (ros_message.attack ? true : false);
  // Member: missing
  cdr << (ros_message.missing ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_iw_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  iw_interfaces::msg::Robot & ros_message)
{
  // Member: label
  cdr >> ros_message.label;

  // Member: faction
  cdr >> ros_message.faction;

  // Member: id
  cdr >> ros_message.id;

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  // Member: hp
  cdr >> ros_message.hp;

  // Member: level
  cdr >> ros_message.level;

  // Member: attack
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.attack = tmp ? true : false;
  }

  // Member: missing
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.missing = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_iw_interfaces
get_serialized_size(
  const iw_interfaces::msg::Robot & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: label
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.label.size() + 1);
  // Member: faction
  {
    size_t item_size = sizeof(ros_message.faction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);
  // Member: hp
  {
    size_t item_size = sizeof(ros_message.hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: level
  {
    size_t item_size = sizeof(ros_message.level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: attack
  {
    size_t item_size = sizeof(ros_message.attack);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: missing
  {
    size_t item_size = sizeof(ros_message.missing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_iw_interfaces
max_serialized_size_Robot(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: label
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: faction
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: level
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: attack
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: missing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = iw_interfaces::msg::Robot;
    is_plain =
      (
      offsetof(DataType, missing) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Robot__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const iw_interfaces::msg::Robot *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Robot__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<iw_interfaces::msg::Robot *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Robot__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const iw_interfaces::msg::Robot *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Robot__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Robot(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Robot__callbacks = {
  "iw_interfaces::msg",
  "Robot",
  _Robot__cdr_serialize,
  _Robot__cdr_deserialize,
  _Robot__get_serialized_size,
  _Robot__max_serialized_size
};

static rosidl_message_type_support_t _Robot__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Robot__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace iw_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_iw_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<iw_interfaces::msg::Robot>()
{
  return &iw_interfaces::msg::typesupport_fastrtps_cpp::_Robot__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, iw_interfaces, msg, Robot)() {
  return &iw_interfaces::msg::typesupport_fastrtps_cpp::_Robot__handle;
}

#ifdef __cplusplus
}
#endif
