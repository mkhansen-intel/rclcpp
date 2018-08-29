// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__CREATE_ACTION_CLIENT_HPP_
#define RCLCPP__CREATE_ACTION_CLIENT_HPP_

#include <string>
#include <memory>
#include <utility>

#include "rclcpp/action_client.hpp"

namespace rclcpp
{

/// Create an Action Client
///
template<typename ActionT, typename MessageT, typename CallbackT, typename Alloc>
typename rclcpp::ActionClient<ActionT, MessageT, CallbackT, Alloc>::SharedPtr
create_action_client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const std::string & action_name,
  CallbackT && feedback_callback,
  rcl_client_options_t & options,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  bool use_intra_process_comms,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  auto action_client = rclcpp::ActionClient<ActionT, MessageT, CallbackT, Alloc>::make_shared(
    node_base,
    node_graph,
    node_services,
    node_topics,
    action_name,
    std::forward<CallbackT>(feedback_callback),
    options,
    group,
    ignore_local_publications,
    use_intra_process_comms,
    msg_mem_strat,
    allocator);

  return action_client;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_ACTION_CLIENT_HPP_
