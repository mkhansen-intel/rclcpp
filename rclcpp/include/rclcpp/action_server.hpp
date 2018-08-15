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

  virtual std::shared_ptr<void> create_request() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;

  virtual void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request) = 0;

  virtual void handle_cancel(
	std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request) = 0;

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

template<typename ServiceT>
class ActionServer : public ActionServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ActionServer)

  ActionServer(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & action_name,
    AnyServiceCallback<ServiceT> request_callback,
    AnyServiceCallback<ServiceT> cancel_callback,
    rcl_service_options_t & service_options)
  : ActionServerBase(node_handle), request_callback_(request_callback), cancel_callback_(cancel_callback)
  {
	  std::string request_service_name = "_request_" + action_name;
    request_service_ = Service<ServiceT>::make_shared(node_handle, request_service_name, request_callback, service_options);
	  std::string cancel_service_name = "_cancel_" + action_name;
    cancel_service_ = Service<ServiceT>::make_shared(node_handle, cancel_service_name, cancel_callback, service_options);
  
    //std::string feedback_topic_name = "_feedback_" + action_name;

    // TODO: Add feedback publisher
    //feedback_publisher_ =
    // TODO: Create simple example / test for Action Server
  }
  
  ActionServer() = delete;

  virtual ~ActionServer()
  {
  }

  std::shared_ptr<void> create_request()
  {
    return request_service_->create_request();
  }

  std::shared_ptr<rmw_request_id_t> create_request_header()
  {
	return request_service_->create_request_header();
  }

  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    request_service_->handle_request(request_header, request);
  }

  void handle_cancel(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    cancel_service_->handle_request(request_header, request);
  }

  void send_request_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    request_service_->send_response(req_id, response);
  }

  void send_cancel_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response)
  {
	cancel_service_->send_response(req_id, response);
  }

private:
  RCLCPP_DISABLE_COPY(ActionServer)
  std::shared_ptr<Service<ServiceT>> request_service_;
  std::shared_ptr<Service<ServiceT>> cancel_service_;
  AnyServiceCallback<ServiceT> request_callback_;
  AnyServiceCallback<ServiceT> cancel_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_
