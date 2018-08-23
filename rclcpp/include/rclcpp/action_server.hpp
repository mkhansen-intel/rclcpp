// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ACTIONSERVER_HPP_
#define RCLCPP__ACTIONSERVER_HPP_

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/service.h"

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/logging.hpp"

#include "rclcpp/service.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/create_publisher.hpp"

namespace rclcpp
{

class ActionServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ActionServerBase)

  RCLCPP_PUBLIC
  explicit ActionServerBase(
    std::shared_ptr<rcl_node_t> node_handle);

  RCLCPP_PUBLIC
  virtual ~ActionServerBase();

  RCLCPP_PUBLIC
  const char *
  get_action_name();

protected:
  RCLCPP_DISABLE_COPY(ActionServerBase)

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  std::shared_ptr<rcl_node_t> node_handle_;
};

template<typename ActionT, typename MessageT, typename Alloc = std::allocator<void>,
	    typename PublisherT = ::rclcpp::Publisher<MessageT, Alloc>>
class ActionServer : public ActionServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ActionServer)

  ActionServer(
    std::shared_ptr<rcl_node_t> node_handle,
	std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
	std::shared_ptr<node_interfaces::NodeTopicsInterface> node_topics,
    const std::string & action_name,
    AnyServiceCallback<ActionT> request_callback,
    AnyServiceCallback<ActionT> cancel_callback,
    rcl_service_options_t & service_options,
	rclcpp::callback_group::CallbackGroup::SharedPtr group,
    bool use_intra_process_comms,
	std::shared_ptr<Alloc> allocator)
  : ActionServerBase(node_handle), action_name_(action_name)
  {
    std::string request_service_name = "_request_" + action_name;
    request_service_ = Service<ActionT>::make_shared(node_handle, request_service_name, request_callback, service_options);
    auto req_base_ptr = std::dynamic_pointer_cast<ServiceBase>(request_service_);
    node_services->add_service(req_base_ptr, group);

    std::string cancel_service_name = "_cancel_" + action_name;
    cancel_service_ = Service<ActionT>::make_shared(node_handle, cancel_service_name, cancel_callback, service_options);
    auto cancel_base_ptr = std::dynamic_pointer_cast<ServiceBase>(cancel_service_);
    node_services->add_service(cancel_base_ptr, group);

    std::string feedback_topic_name = "_feedback_" + action_name;

    feedback_publisher_ = rclcpp::create_publisher<MessageT, Alloc, PublisherT>(
    	    node_topics.get(),
    	    feedback_topic_name,
    	    service_options.qos,
    	    use_intra_process_comms,
    	    allocator);
  }
  
  ActionServer() = delete;

  virtual ~ActionServer()
  {
  }

  const char *
  get_action_name()
  {
	  return action_name_;
  }

  void publish_feedback(const std::shared_ptr<MessageT> & msg)
  {
	  feedback_publisher_->publish(msg);
  }

  void publish_feedback(const MessageT & msg)
  {
	  feedback_publisher_->publish(msg);
  }

private:
  RCLCPP_DISABLE_COPY(ActionServer)
  const std::string action_name_;
  std::shared_ptr<Service<ActionT>> request_service_;
  std::shared_ptr<Service<ActionT>> cancel_service_;
  std::shared_ptr<Publisher<MessageT>> feedback_publisher_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_
