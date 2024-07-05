// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_ROS_ACTION_STATE_HPP
#define YASMIN_ROS_ACTION_STATE_HPP

#include <condition_variable>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::placeholders;

namespace yasmin_ros {

template <typename ActionT> class ActionState : public yasmin::State {

  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result::SharedPtr;

  using SendGoalOptions =
      typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  using ActionClient = typename rclcpp_action::Client<ActionT>::SharedPtr;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  using CreateGoalHandler =
      std::function<Goal(std::shared_ptr<yasmin::blackboard::Blackboard>)>;
  using ResutlHandler = std::function<std::string(
      std::shared_ptr<yasmin::blackboard::Blackboard>, Result)>;

public:
  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              std::vector<std::string> outcomes)
      : ActionState(action_name, create_goal_handler, outcomes, nullptr) {}

  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              ResutlHandler result_handler = nullptr)
      : ActionState(action_name, create_goal_handler, {}, result_handler) {}

  ActionState(std::string action_name, CreateGoalHandler create_goal_handler,
              std::vector<std::string> outcomes,
              ResutlHandler result_handler = nullptr)
      : ActionState(nullptr, action_name, create_goal_handler, outcomes,
                    result_handler) {}

  ActionState(rclcpp::Node *node, std::string action_name,
              CreateGoalHandler create_goal_handler,
              std::vector<std::string> outcomes,
              ResutlHandler result_handler = nullptr)
      : State({}) {

    this->outcomes = {basic_outcomes::SUCCEED, basic_outcomes::ABORT,
                      basic_outcomes::CANCEL};

    if (outcomes.size() > 0) {
      for (std::string outcome : outcomes) {
        this->outcomes.push_back(outcome);
      }
    }

    if (node == nullptr) {
      this->node = YasminNode::get_instance();
    } else {
      this->node = node;
    }

    this->action_client =
        rclcpp_action::create_client<ActionT>(this->node, action_name);

    this->create_goal_handler = create_goal_handler;
    this->result_handler = result_handler;

    if (this->create_goal_handler == nullptr) {
      throw std::invalid_argument("create_goal_handler is needed");
    }
  }

  void cancel_state() {
    // this->action_client->cancel_goal();
    yasmin::State::cancel_state();
  }

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    std::unique_lock<std::mutex> lock(this->action_done_mutex);

    Goal goal = this->create_goal_handler(blackboard);

    this->action_client->wait_for_action_server();

    // options
    auto send_goal_options = SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionState::goal_response_callback, this, _1);

    send_goal_options.result_callback =
        std::bind(&ActionState::result_callback, this, _1);

    this->action_client->async_send_goal(goal, send_goal_options);

    // wait for results
    this->action_done_cond.wait(lock);

    if (this->action_status == rclcpp_action::ResultCode::CANCELED) {
      return basic_outcomes::CANCEL;

    } else if (this->action_status == rclcpp_action::ResultCode::ABORTED) {
      return basic_outcomes::ABORT;

    } else if (this->action_status == rclcpp_action::ResultCode::SUCCEEDED) {

      if (this->result_handler != nullptr) {
        return this->result_handler(blackboard, this->action_result);
      }

      return basic_outcomes::SUCCEED;
    }

    return basic_outcomes::ABORT;
  }

private:
  rclcpp::Node *node;

  ActionClient action_client;
  std::condition_variable action_done_cond;
  std::mutex action_done_mutex;

  Result action_result;
  rclcpp_action::ResultCode action_status;

  std::shared_ptr<GoalHandle> goal_handle;
  std::mutex goal_handle_mutex;

  CreateGoalHandler create_goal_handler;
  ResutlHandler result_handler;

  void
  goal_response_callback(const typename GoalHandle::SharedPtr &goal_handle) {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    this->goal_handle = goal_handle;
  }

  void result_callback(const typename GoalHandle::WrappedResult &result) {
    this->action_result = result.result;
    this->action_status = result.code;
    this->action_done_cond.notify_one();
  }
};

} // namespace yasmin_ros

#endif
