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

#ifndef SRC__ROBOT_HPP
#define SRC__ROBOT_HPP

#include <QtCore/QObject>
#include <QtCore/QEvent>

#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>
#include <nodes/NodeData>

#include"../datatypes/StringData.hpp"
#include"../datatypes/WheelData.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <thread>
#include <memory>
#include <mutex>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::Connection;

///=============================================================================
// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class Robot : public NodeDataModel, public std::enable_shared_from_this<Robot>
{
  Q_OBJECT

public:
  Robot();

  ~Robot();

  QString
  caption() const override
  { return QString("Robot"); }

  bool
  captionVisible() const override { return true; }

  static QString
  Name()
  { return QString("Robot"); }

  QString
  name() const override
  { return Robot::name(); }

  unsigned int
  nPorts(PortType portType) const override;

  bool
  portCaptionVisible(PortType portType, PortIndex portIndex) const override
  {
    Q_UNUSED(portType); Q_UNUSED(portIndex);
    return true;
  }

  QString portCaption(PortType portType, PortIndex portIndex) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData> outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData> nodeData, PortIndex port) override;

  QWidget* embeddedWidget() override;

  void inputConnectionDeleted(Connection const& con) override;

  bool resizable() const override { return true; }

protected:

  bool eventFilter(QObject *object, QEvent *event) override;

private:
  using Twist = geometry_msgs::msg::TwistStamped;
  struct Data
  {
    QLabel* _label;
    std::shared_ptr<StringData> _string_data;
    std::shared_ptr<WheelData> _wheel_1_data = nullptr;
    std::shared_ptr<WheelData> _wheel_2_data = nullptr;
    std::shared_ptr<StringData> _joystick_data = nullptr;
    std::thread _spin_thread;
    std::thread _pub_thread;
    std::chrono::nanoseconds _period;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<Twist>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;
    std::mutex _mutex;
  };

  std::shared_ptr<Data> _data;


};

#endif // SRC__ROBOT_HPP