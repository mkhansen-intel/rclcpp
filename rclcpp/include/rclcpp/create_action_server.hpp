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

#ifndef RCLCPP__CREATE_ACTION_SERVER_HPP_
#define RCLCPP__CREATE_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/action_server.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{

/// Create an Action with a given type.
/// \internal
template<typename ActionT, typename CallbackT>
typename rclcpp::ActionServer<ActionT>::SharedPtr
create_action_server(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & action_name,
  CallbackT && action_callback,
  CallbackT && cancel_callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos_profile;

  rclcpp::AnyServiceCallback<ActionT> action_service_callback;
  action_service_callback.set(std::forward<CallbackT>(action_callback));

  rclcpp::AnyServiceCallback<ActionT> cancel_service_callback;
  cancel_service_callback.set(std::forward<CallbackT>(cancel_callback));

  auto action_server = rclcpp::ActionServer<ActionT>::make_shared(node_base->get_shared_rcl_node_handle(),
		  action_name, action_service_callback, cancel_service_callback, service_options,
		  node_services, group);

  return action_server;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SERVICE_HPP_
