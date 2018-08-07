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

#ifndef RCLCPP__SERVICE_HPP_
#define RCLCPP__SERVICE_HPP_

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
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

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

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_service_t>
  get_request_service_handle();

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_service_t>
  get_request_service_handle() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_service_t>
  get_cancel_service_handle();

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_service_t>
  get_cancel_service_handle() const;

  RCLCPP_PUBLIC
  rcl_publisher_t *
  get_publisher_handle();

  RCLCPP_PUBLIC
  const rcl_publisher_t *
  get_publisher_handle() const;

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

  std::shared_ptr<rcl_service_t> request_service_handle_;
  std::shared_ptr<rcl_service_t> cancel_service_handle_;
  std::shared_ptr<rcl_publisher_t>

  bool owns_rcl_handle_ = true;
};

template<typename ActionT>
class ActionServer : public ActionServerBase
{
public:
  using CallbackType = std::function<
    void (
      const std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;

  using CallbackWithHeaderType = std::function<
    void (
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  ActionServer(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & action_name,
    AnyServiceCallback<ServiceT> request_callback,
    AnyServiceCallback<ServiceT> cancel_callback,
    rcl_service_options_t & service_options)
  : ActionServerBase(node_handle), request_callback_(request_callback), cancel_callback_(cancel_callback)
  {
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle = get_service_type_support_handle<ServiceT>();

    std::weak_ptr<rcl_node_t> weak_node_handle(node_handle_);
    // rcl does the static memory allocation here
    request_handle_ = std::shared_ptr<rcl_service_t>(
      new rcl_service_t, [weak_node_handle](rcl_service_t * service)
      {
        auto handle = weak_node_handle.lock();
        if (handle) {
          if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
            RCLCPP_ERROR(
              rclcpp::get_logger(rcl_node_get_logger_name(handle.get())).get_child("rclcpp"),
              "Error in destruction of rcl service handle: %s",
              rcl_get_error_string_safe());
            rcl_reset_error();
          }
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Error in destruction of rcl service handle: "
            "the Node Handle was destructed too early. You will leak memory");
        }
        delete service;
      });
    *request_handle_.get() = rcl_get_zero_initialized_service();

    rcl_ret_t ret = rcl_service_init(
      request_handle_.get(),
      node_handle.get(),
      service_type_support_handle,
      service_name.c_str(),
      &service_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        expand_topic_or_service_name(
          service_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
      }

      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
    }
  }

  ActionServer(
    std::shared_ptr<rcl_node_t> node_handle,
    std::shared_ptr<rcl_service_t> request_handle,
    AnyServiceCallback<ServiceT> request_callback)
  : ActionServerBase(node_handle),
    request_callback_(request_callback)
  {
    // check if service handle was initialized
    if (!rcl_service_is_valid(request_handle.get(), nullptr)) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("rcl_service_t in constructor argument must be initialized beforehand."));
      // *INDENT-ON*
    }

    request_handle_ = request_handle;
  }

  ActionServer(
    std::shared_ptr<rcl_node_t> node_handle,
    rcl_service_t * request_handle,
    AnyServiceCallback<ServiceT> request_callback),
    AnyServiceCallback<ServiceT> cancel_callback)
  : ActionServerBase(node_handle),
    request_callback_(request_callback)
  {
    // check if service handle was initialized
    if (!rcl_service_is_valid(request_handle, nullptr)) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("rcl_service_t in constructor argument must be initialized beforehand."));
      // *INDENT-ON*
    }

    // In this case, rcl owns the service handle memory
    request_handle_ = std::shared_ptr<rcl_service_t>(new rcl_service_t);
    request_handle_->impl = request_handle->impl;
  }

  ActionServer() = delete;

  virtual ~ActionServer()
  {
  }

  std::shared_ptr<void> create_request()
  {
    return std::shared_ptr<void>(new typename ServiceT::Request());
  }

  std::shared_ptr<rmw_request_id_t> create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    request_callback_.dispatch(request_header, typed_request, response);
    send_response(request_header, response);
  }

  void handle_cancel(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    cancel_callback_.dispatch(request_header, typed_request, response);
    send_response(request_header, response);
  }

  void send_request_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    rcl_ret_t status = rcl_send_response(get_request_service_handle().get(), req_id.get(), response.get());

    if (status != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to send response");
    }
  }

  void send_cancel_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    rcl_ret_t status = rcl_send_response(get_cancel_service_handle().get(), req_id.get(), response.get());

    if (status != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to send response");
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  AnyServiceCallback<ServiceT> request_callback_;
  AnyServiceCallback<ServiceT> cancel_callback_;

};

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_
