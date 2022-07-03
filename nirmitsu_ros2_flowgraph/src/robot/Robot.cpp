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

  _data->_active_frame_id = QString();
  _data->_string_data = std::make_shared<StringData>();

  _data->_period = std::chrono::milliseconds(67);

  _data->_node = std::make_shared<rclcpp::Node>("robot_node");
  _data->_pub = _data->_node->create_publisher<Twist>(
    "/cmd_vel",
    rclcpp::SystemDefaultsQoS().reliable()
  );

  _data->_pub_thread = std::thread(
    [data = _data]()
    {
      while (rclcpp::ok())
      {
        // TODO(YV): Wait on a cv for input from wheels
        std::lock_guard<std::mutex>lock(data->_mutex);
        {
          if (data->_left_wheel_data == nullptr && data->_right_wheel_data == nullptr)
          {
            RCLCPP_DEBUG(
              data->_node->get_logger(),
              "[Robot] Waiting for a wheel to connect"
            );
            std::this_thread::sleep_for(data->_period);
            continue;
          }
          auto msg = std::make_unique<Twist>();
          msg->header.stamp = data->_node->get_clock()->now();
          if (data->_left_wheel_data != nullptr)
          {
            if (data->_right_wheel_data != nullptr)
            {
              // Both wheels are connected
              RCLCPP_DEBUG(
                data->_node->get_logger(),
                "[Robot] Logic not implemented for two wheels connected"
              );
              std::this_thread::sleep_for(data->_period);
              continue;
            }
            else
            {
              const auto& value = data->_left_wheel_data->value();
              msg->header.frame_id = value.name.toStdString();
              msg->twist.linear.x = value.speed / 100.0;
              data->_active_frame_id = value.name;
            }
          }
          else
          {
            if (data->_left_wheel_data != nullptr)
            {
              // Both wheels are connected
              RCLCPP_DEBUG(
                data->_node->get_logger(),
                "[Robot] Logic not implemented for two wheels connected"
              );
              std::this_thread::sleep_for(data->_period);
              continue;
            }
            else
            {
              const auto& value = data->_right_wheel_data->value();
              msg->header.frame_id = value.name.toStdString();
              msg->twist.linear.x = value.speed / 100.0;
              data->_active_frame_id = value.name;
            }
          }
          // data->_string_data->value(
          //   QStringLiteral("[%1_%2] Published Speed %2 to Wheel %3")
          //   .arg((int)msg->header.stamp.sec)
          //   .arg((int)msg->header.stamp.nanosec)
          //   .arg((int)msg->twist.linear.x)
          //   .arg(data->_active_frame_id)
          // );
          data->_pub->publish(std::move(msg));
        }
        std::this_thread::sleep_for(data->_period);
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
      result = 3;
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
        return QStringLiteral("Left Wheel");
      else if (portIndex == 1)
        return QStringLiteral("Right Wheel");
      else if (portIndex == 2)
        return QStringLiteral("Joystick");
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
      return _data->_string_data->type();
    }
    else
    {
      return NodeDataType();
    }
  }

  else if (portType == PortType::Out)
  {
    return _data->_string_data->type();
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
  // Left Wheel
  if (port == 0)
  {
    auto wheel_data = std::dynamic_pointer_cast<WheelData>(data);
    if (wheel_data == nullptr)
      return;
    // First time connecting to Left Wheel
    if (_data->_left_wheel_data == nullptr)
    {
      _data->_left_wheel_data = std::make_shared<WheelData>();
    }
    _data->_left_wheel_data = std::move(wheel_data);
    _data->_string_data->value(
      QStringLiteral("Updated Left wheel:\n") +
      _data->_left_wheel_data->to_string());

  }
  // Right Wheel
  else if (port == 1)
  {
    auto wheel_data = std::dynamic_pointer_cast<WheelData>(data);
    if (wheel_data == nullptr)
      return;
    // First time connecting to Right Wheel
    if (_data->_right_wheel_data == nullptr)
    {
      _data->_right_wheel_data = std::make_shared<WheelData>();
    }
    _data->_right_wheel_data = std::move(wheel_data);
    _data->_string_data->value(
      QStringLiteral("Updated right wheel:\n") +
      _data->_right_wheel_data->to_string());  }
  // Joystick
  else if (port == 2)
  {

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
      "Deleted left wheel node"
    );
    _data->_left_wheel_data = nullptr;
  }
  else if (port == 1)
  {
    RCLCPP_INFO(
      _data->_node->get_logger(),
      "Deleted right wheel node"
    );
    // std::lock_guard<std::mutex>lock(_data->_mutex);
    _data->_right_wheel_data = nullptr;
  }
  else
    return;
}

//=============================================================================
bool Robot::eventFilter(QObject *object, QEvent *event)
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