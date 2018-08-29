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

#ifndef RCLCPP__ACTION_CLIENT_HPP_
#define RCLCPP__ACTION_CLIENT_HPP_

#include <string>
#include <memory>
#include <utility>

#include "rclcpp/logger.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/create_subscription.hpp"

namespace rclcpp
{

namespace node_interfaces
{
class NodeBaseInterface;
}  // namespace node_interfaces

template<typename ActionT, typename MessageT, typename CallbackT, typename Alloc>
class ActionClient
{
public:
  using SharedRequest = typename ActionT::Request::SharedPtr;
  using SharedResponse = typename ActionT::Response::SharedPtr;
  using SharedFuture = std::shared_future<SharedResponse>;

  RCLCPP_SMART_PTR_DEFINITIONS(ActionClient)

  ActionClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const std::string & action_name,
    CallbackT && feedback_callback,
    rcl_client_options_t & client_options,
    rclcpp::callback_group::CallbackGroup::SharedPtr group,
    bool ignore_local_publications,
    bool use_intra_process_comms,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat,
    std::shared_ptr<Alloc> allocator)
  {
    std::string request_service_name = "_request_" + action_name;
    request_client_ = Client<ActionT>::make_shared(
      node_base.get(),
      node_graph,
      request_service_name,
      client_options);

    auto action_base_ptr = std::dynamic_pointer_cast<ClientBase>(request_client_);
    node_services->add_client(action_base_ptr, group);

    std::string cancel_service_name = "_cancel_" + action_name;
    cancel_client_ = Client<ActionT>::make_shared(
      node_base.get(),
      node_graph,
      cancel_service_name,
      client_options);

    auto cancel_base_ptr = std::dynamic_pointer_cast<ClientBase>(cancel_client_);
    node_services->add_client(cancel_base_ptr, group);

    RCLCPP_INFO(rclcpp::get_logger(action_name), "DEBUG: feedback_callback = %x", feedback_callback)

    std::string feedback_topic_name = "_feedback_" + action_name;
    using CallbackMessageT =
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type;
    feedback_subscriber_ =
      rclcpp::create_subscription<MessageT, CallbackT, Alloc, CallbackMessageT>(
      node_topics.get(),
      feedback_topic_name,
      std::forward<CallbackT>(feedback_callback),
      client_options.qos,
      group,
      ignore_local_publications,
      use_intra_process_comms,
      msg_mem_strat,
      allocator);
  }

  virtual ~ActionClient()
  {
  }

  template<typename RatioT = std::milli>
  bool
  wait_for_action(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return request_client_->wait_for_service(timeout);
  }

  SharedFuture
  async_send_request(SharedRequest request)
  {
    return request_client_->async_send_request(request);
  }

  SharedFuture
  cancel_request(SharedRequest request)
  {
    return cancel_client_->async_send_request(request);
  }

private:
  RCLCPP_DISABLE_COPY(ActionClient)
  std::shared_ptr<Client<ActionT>> request_client_;
  std::shared_ptr<Client<ActionT>> cancel_client_;
  std::shared_ptr<Subscription<MessageT>> feedback_subscriber_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ACTION_CLIENT_HPP_
