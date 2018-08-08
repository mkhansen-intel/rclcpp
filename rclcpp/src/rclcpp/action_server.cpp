// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/action_server.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"

using rclcpp::ActionServerBase;

ActionServerBase::ActionServerBase(std::shared_ptr<rcl_node_t> node_handle)
: node_handle_(node_handle)
{}

ActionServerBase::~ActionServerBase()
{}

const char *
ActionServerBase::get_action_name()
{
  return rcl_service_get_service_name(this->get_request_service_handle().get());
}

std::shared_ptr<rcl_service_t>
ActionServerBase::get_request_service_handle()
{
  return request_service_.get_service_handle();
}

std::shared_ptr<const rcl_service_t>
ActionServerBase::get_request_service_handle() const
{
  return request_service_.get_service_handle();
}

rcl_node_t *
ActionServerBase::get_rcl_node_handle()
{
  return node_handle_.get();
}

const rcl_node_t *
ActionServerBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}
