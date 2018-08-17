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

#ifndef RCLCPP__ACTION_CLIENT_HPP_
#define RCLCPP__ACTION_CLIENT_HPP_

#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/wait.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{

namespace node_interfaces
{
class NodeBaseInterface;
}  // namespace node_interfaces

class ActionClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ActionClientBase)

  RCLCPP_PUBLIC
  ActionClientBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph);

  RCLCPP_PUBLIC
  virtual ~ActionClientBase();

  RCLCPP_PUBLIC
  const char *
  get_action_name() const;

  RCLCPP_PUBLIC
  bool
  action_is_ready() const;

  template<typename RatioT = std::milli>
  bool
  wait_for_action(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;
  virtual void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

protected:
  RCLCPP_DISABLE_COPY(ActionClientBase)

  RCLCPP_PUBLIC
  bool
  wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  std::shared_ptr<rcl_node_t> node_handle_;

  std::shared_ptr<rcl_client_t> client_handle_;
};

template<typename ActionT>
class ActionClient : public ActionClientBase
{
public:
  using SharedRequest = typename ActionT::Request::SharedPtr;
  using SharedResponse = typename ActionT::Response::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using CallbackType = std::function<void(SharedFuture)>;
  using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(ActionClient)

  ActionClient(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & action_name,
    rcl_client_options_t & client_options,
	std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
	rclcpp::callback_group::CallbackGroup::SharedPtr group)
  : ActionClientBase(node_base, node_graph)
  {
    std::string request_service_name = "_request_" + action_name;
    request_client_ = Client<ActionT>::make_shared(
        node_base,
        node_graph,
        request_service_name,
        client_options);

    auto action_base_ptr = std::dynamic_pointer_cast<ClientBase>(request_client_);
    node_services->add_client(action_base_ptr, group);

    std::string cancel_service_name = "_cancel_" + action_name;
    cancel_client_ = Client<ActionT>::make_shared(
        node_base,
        node_graph,
        request_service_name,
        client_options);

    auto cancel_base_ptr = std::dynamic_pointer_cast<ClientBase>(cancel_client_);
    node_services->add_client(cancel_base_ptr, group);
}

  virtual ~ActionClient()
  {
  }
/*
  std::shared_ptr<void>
  create_response()
  {
    return std::shared_ptr<void>(new typename ActionT::Response());
  }

  std::shared_ptr<rmw_request_id_t>
  create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void
  handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response)
  {
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    auto typed_response = std::static_pointer_cast<typename ActionT::Response>(response);
    int64_t sequence_number = request_header->sequence_number;
    // TODO(esteve) this should throw instead since it is not expected to happen in the first place
    if (this->pending_requests_.count(sequence_number) == 0) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Received invalid sequence number. Ignoring...");
      return;
    }
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    // Unlock here to allow the service to be called recursively from one of its callbacks.
    lock.unlock();

    call_promise->set_value(typed_response);
    callback(future);
  }

  SharedFuture
  async_send_request(SharedRequest request)
  {
    return async_send_request(request, [](SharedFuture) {});
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackType
      >::value
    >::type * = nullptr
  >
  SharedFuture
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number;
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request.get(), &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    pending_requests_[sequence_number] =
      std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
    return f;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackWithRequestType
      >::value
    >::type * = nullptr
  >
  SharedFutureWithRequest
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    action_client_->async_send_request(request, cb)

    return future_with_request;
  }
*/
private:
  RCLCPP_DISABLE_COPY(ActionClient)
  std::shared_ptr<Client<ActionT>> request_client_;
  std::shared_ptr<Client<ActionT>> cancel_client_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ACTION_CLIENT_HPP_
