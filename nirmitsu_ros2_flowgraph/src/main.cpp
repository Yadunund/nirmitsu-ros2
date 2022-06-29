/*
 * Copyright (C) 2022 Yadunund Vijay
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <nodes/NodeData>
#include <nodes/FlowScene>
#include <nodes/FlowView>
#include <nodes/DataModelRegistry>

#include <QtWidgets/QApplication>

#include "SimplePublisher.hpp"

#include <rclcpp/rclcpp.hpp>

using QtNodes::DataModelRegistry;
using QtNodes::FlowScene;
using QtNodes::FlowView;

static std::shared_ptr<DataModelRegistry> registerDataModels()
{
  auto ret = std::make_shared<DataModelRegistry>();
  ret->registerModel<SimplePublisher>();
  return ret;
}


int main(int argc, char** argv)
{

  // std::vector<std::string> non_ros_args =
  //   rclcpp::init_and_remove_ros_arguments(argc, argv);
  // std::vector<char *> non_ros_args_c_strings;
  // for (auto & arg : non_ros_args) {
  //   non_ros_args_c_strings.push_back(&arg.front());
  // }
  // int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

  // QApplication app(non_ros_argc, non_ros_args_c_strings.data());

  QApplication app(argc, argv);

  rclcpp::init(argc, argv);

  FlowScene scene(registerDataModels());

  FlowView view(&scene);

  view.setWindowTitle("Node-based flow editor");
  view.resize(800, 600);
  view.show();

  return app.exec();
  rclcpp::shutdown();
}
