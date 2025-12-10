// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from example_interfaces:msg/ExampleMsg.idl
// generated code does not contain a copyright notice

#ifndef EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__BUILDER_HPP_
#define EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "example_interfaces/msg/detail/example_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace example_interfaces
{

namespace msg
{

namespace builder
{

class Init_ExampleMsg_id
{
public:
  explicit Init_ExampleMsg_id(::example_interfaces::msg::ExampleMsg & msg)
  : msg_(msg)
  {}
  ::example_interfaces::msg::ExampleMsg id(::example_interfaces::msg::ExampleMsg::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::example_interfaces::msg::ExampleMsg msg_;
};

class Init_ExampleMsg_message
{
public:
  Init_ExampleMsg_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExampleMsg_id message(::example_interfaces::msg::ExampleMsg::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExampleMsg_id(msg_);
  }

private:
  ::example_interfaces::msg::ExampleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::example_interfaces::msg::ExampleMsg>()
{
  return example_interfaces::msg::builder::Init_ExampleMsg_message();
}

}  // namespace example_interfaces

#endif  // EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__BUILDER_HPP_
