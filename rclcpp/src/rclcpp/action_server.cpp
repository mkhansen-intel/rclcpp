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

using rclcpp::ActionServerBase;

ActionServerBase::ActionServerBase(std::shared_ptr<rcl_node_t> node_handle)
: node_handle_(node_handle)
{}

ActionServerBase::~ActionServerBase()
{}

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
