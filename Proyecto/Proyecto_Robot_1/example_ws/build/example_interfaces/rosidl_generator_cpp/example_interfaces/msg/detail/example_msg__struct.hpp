// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from example_interfaces:msg/ExampleMsg.idl
// generated code does not contain a copyright notice

#ifndef EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_HPP_
#define EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__example_interfaces__msg__ExampleMsg __attribute__((deprecated))
#else
# define DEPRECATED__example_interfaces__msg__ExampleMsg __declspec(deprecated)
#endif

namespace example_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExampleMsg_
{
  using Type = ExampleMsg_<ContainerAllocator>;

  explicit ExampleMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
      this->id = 0l;
    }
  }

  explicit ExampleMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
      this->id = 0l;
    }
  }

  // field types and members
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _id_type =
    int32_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    example_interfaces::msg::ExampleMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const example_interfaces::msg::ExampleMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      example_interfaces::msg::ExampleMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      example_interfaces::msg::ExampleMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__example_interfaces__msg__ExampleMsg
    std::shared_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__example_interfaces__msg__ExampleMsg
    std::shared_ptr<example_interfaces::msg::ExampleMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExampleMsg_ & other) const
  {
    if (this->message != other.message) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExampleMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExampleMsg_

// alias to use template instance with default allocator
using ExampleMsg =
  example_interfaces::msg::ExampleMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace example_interfaces

#endif  // EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_HPP_
