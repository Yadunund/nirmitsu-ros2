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

#include "outputs/Display.hpp"
#include "inputs/NumberSlider.hpp"
#include "inputs/TextBox.hpp"
#include "robot/Robot.hpp"
#include "robot/RobotWheel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>

using QtNodes::DataModelRegistry;
using QtNodes::FlowScene;
using QtNodes::FlowView;

static std::shared_ptr<DataModelRegistry> registerDataModels()
{
  auto ret = std::make_shared<DataModelRegistry>();

  // Register robots
  ret->registerModel<Robot>("Robot");
  ret->registerModel<RobotWheel>("Robot");

  // Register outputs
  ret->registerModel<Display>("Result");

  // Register inputs
  ret->registerModel<NumberSlider>("Input");
  ret->registerModel<TextBox>("Input");

  return ret;
}


int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  rclcpp::init(argc, argv);

  FlowScene scene(registerDataModels());

  FlowView view(&scene);

  view.setWindowTitle("Node-based flow editor");
  view.resize(800, 600);
  view.show();

  app.exec();
  rclcpp::shutdown();
}
