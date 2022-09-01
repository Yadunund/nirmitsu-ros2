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

#include "Robot.hpp"

//=============================================================================
Robot::Robot()
{
  _data = std::make_shared<Data>();
  _data->_label = new QLabel("MyRobot");
  _data->_label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

  _data->_string_data = std::make_shared<StringData>();
  _data->_on_data = std::make_shared<BoolData>(false);

  _data->_period = std::chrono::milliseconds(250);

  _data->_node = std::make_shared<rclcpp::Node>("robot_node");
  _data->_pub = _data->_node->create_publisher<Twist>(
    "/cmd_vel",
    rclcpp::QoS(10).reliable()
  );

  _data->_pub_thread = std::thread(
    [data = _data, me = weak_from_this()]()
    {
      while (rclcpp::ok())
      {
        std::this_thread::sleep_for(data->_period);
        // TODO(YV): Wait on a cv for input from wheels
        std::lock_guard<std::mutex> lock(data->_mutex);
        if (data->_wheel_1_data == nullptr && data->_wheel_2_data == nullptr)
        {
          RCLCPP_DEBUG(
            data->_node->get_logger(),
            "[Robot] Waiting for a wheel to connect"
          );
          continue;
        }

        // If we're receiving joystick commands, only publish that Twist
        if (data->_joystick_data != nullptr &&
        data->_wheel_1_data != nullptr &&
        data->_wheel_2_data != nullptr)
        {
          auto msg = std::make_unique<Twist>();
          msg->header.stamp = data->_node->get_clock()->now();
          msg->header.frame_id = "robot";
          data->_pub->publish(std::move(msg));
          continue;
        }

        // Publish Wheel msgs
        if (data->_wheel_1_data != nullptr)
        {
          auto msg = std::make_unique<Twist>();
          msg->header.stamp = data->_node->get_clock()->now();
          const auto& value = data->_wheel_1_data->value();
          const int dir = value.reverse ? -1 : 1;
          msg->header.frame_id = value.name.toStdString();
          msg->twist.linear.x =
          data->_on_data->value() ? dir * value.speed / 100.0 : 0.0;
          data->_pub->publish(std::move(msg));

        }

        if (data->_wheel_2_data != nullptr)
        {
          auto msg = std::make_unique<Twist>();
          msg->header.stamp = data->_node->get_clock()->now();
          const auto& value = data->_wheel_2_data->value();
          const int dir = value.reverse ? -1 : 1;
          msg->header.frame_id = value.name.toStdString();
          msg->twist.linear.x =
          data->_on_data->value() ? dir * value.speed / 100.0 : 0.0;
          data->_pub->publish(std::move(msg));
        }

      }
    });

  _data->_spin_thread = std::thread(
    [data = _data]()
    {
      rclcpp::spin(data->_node);
    }
  );

}

//=============================================================================
Robot::~Robot()
{
  if (_data->_spin_thread.joinable())
    _data->_spin_thread.join();
  if (_data->_pub_thread.joinable())
    _data->_pub_thread.join();
}

//=============================================================================
unsigned int Robot::nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 4;
      break;

    case PortType::Out:
      result = 1;
      break;
    default:
      break;
  }

  return result;
}

//=============================================================================
QString Robot::portCaption(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
    if (portIndex == 0)
      return QStringLiteral("Wheel #1");
    else if (portIndex == 1)
      return QStringLiteral("Wheel #2");
    else if (portIndex == 2)
      return QStringLiteral("Joystick");
    else if (portIndex == 3)
      return QStringLiteral("On");
    else
      return QString();
  }
  else if (portType == PortType::Out)
  {
    if (portIndex == 0)
      return QStringLiteral("Display");
    else
      return QString();
  }
  else
  {
    return QString();
  }
}

//=============================================================================
NodeDataType Robot::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
    // Left Wheel
    if (portIndex == 0)
    {
      return WheelData().type();
    }
    // Right Wheel
    else if (portIndex == 1)
    {
      return WheelData().type();
    }
    // Joystick
    else if (portIndex == 2)
    {
      return StringData().type();
    }
    // On
    else if (portIndex == 3)
    {
      return BoolData().type();
    }
    else
    {
      return NodeDataType();
    }
  }

  else if (portType == PortType::Out)
  {
    return StringData().type();
  }
  else
  {
    return NodeDataType();
  }
}

//=============================================================================
void Robot::setInData(std::shared_ptr<NodeData> data, PortIndex port)
{
  // std::lock_guard<std::mutex>lock(_data->_mutex);
  // Wheel 1
  if (port == 0)
  {
    auto wheel_data = std::dynamic_pointer_cast<WheelData>(data);
    if (wheel_data == nullptr)
      return;
    _data->_wheel_1_data = std::move(wheel_data);
    _data->_string_data->value(
      QStringLiteral("Updated wheel:\n") +
      _data->_wheel_1_data->to_string());

  }
  // Wheel 2
  else if (port == 1)
  {
    auto wheel_data = std::dynamic_pointer_cast<WheelData>(data);
    if (wheel_data == nullptr)
      return;
    _data->_wheel_2_data = std::move(wheel_data);
    _data->_string_data->value(
      QStringLiteral("Updated wheel:\n") +
      _data->_wheel_2_data->to_string());
  }
  // Joystick
  else if (port == 2)
  {
    auto joy_data = std::dynamic_pointer_cast<StringData>(data);
    if (joy_data == nullptr)
      return;
    // First time connecting to Right Wheel
    _data->_joystick_data = std::move(joy_data);
    _data->_string_data->value(
      QStringLiteral("Received joystick command %1:\n")
      .arg(_data->_joystick_data->to_string()));
  }
  // On
  else if (port == 3)
  {
    auto on_data = std::dynamic_pointer_cast<BoolData>(data);
    if (data == nullptr)
      return;
    _data->_on_data = on_data;
    _data->_string_data->value(
      QStringLiteral("Motor On changed to %1:\n")
      .arg(_data->_on_data->to_string()));
  }
  else
  {
    return;
  }
  Q_EMIT dataUpdated(0);

}

//=============================================================================
void Robot::inputConnectionDeleted(Connection const& con)
{
  const auto& port = con.getPortIndex(PortType::In);
  if (port == 0)
  {
    // std::lock_guard<std::mutex>lock(_data->_mutex);
    RCLCPP_INFO(
      _data->_node->get_logger(),
      "Deleted wheel #1 node"
    );
    _data->_wheel_1_data = nullptr;
  }
  else if (port == 1)
  {
    RCLCPP_INFO(
      _data->_node->get_logger(),
      "Deleted wheel #2 node"
    );
    // std::lock_guard<std::mutex>lock(_data->_mutex);
    _data->_wheel_2_data = nullptr;
  }
  else if (port == 2)
  {
    RCLCPP_INFO(
      _data->_node->get_logger(),
      "Deleted joystick node"
    );
    // std::lock_guard<std::mutex>lock(_data->_mutex);
    _data->_joystick_data = nullptr;
  }
  else if (port == 3)
  {
    RCLCPP_INFO(
      _data->_node->get_logger(),
      "Deleted On node"
    );
    // std::lock_guard<std::mutex>lock(_data->_mutex);
    _data->_on_data->value(false);
  }
  else
    return;
}

//=============================================================================
bool Robot::eventFilter(QObject* object, QEvent* event)
{
  return false;
}

//=============================================================================
std::shared_ptr<NodeData> Robot::outData(PortIndex)
{
  return _data->_string_data;
}

//=============================================================================
QWidget* Robot::embeddedWidget()
{
  if (_data != nullptr)
    return _data->_label;
  return nullptr;
}
