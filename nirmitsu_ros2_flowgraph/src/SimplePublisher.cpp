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

#include "SimplePublisher.hpp"

#include <QtCore/QEvent>
#include <QtCore/QDir>

#include <QtWidgets/QFileDialog>

#include <iostream>

//=============================================================================
SimplePublisher::SimplePublisher()
{
  _data = std::make_shared<Data>();
  _data->_label = new QLabel("Robot");
  _data->_label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
  _data->_period = std::chrono::seconds(1);

  _data->_node = std::make_shared<rclcpp::Node>("simple_publisher_node");
  _data->_pub = _data->_node->create_publisher<std_msgs::msg::String>(
    "/chatter",
    rclcpp::SystemDefaultsQoS()
  );

  _data->_pub_thread = std::thread(
    [data = _data]()
    {
      while (rclcpp::ok())
      {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello from flow component";
        data->_pub->publish(std::move(msg));
        std::this_thread::sleep_for(data->_period);

      }
    });

  _data->_spin_thread = std::thread(
    [data = _data]()
    {
      while(rclcpp::ok())
      {
        rclcpp::spin_some(data->_node);
      }
    }
  );

}

//=============================================================================
SimplePublisher::~SimplePublisher()
{
  if (_data->_spin_thread.joinable())
    _data->_spin_thread.join();
  if (_data->_pub_thread.joinable())
    _data->_pub_thread.join();
}

unsigned int
SimplePublisher::
nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 1;

    default:
      break;
  }

  return result;
}

//=============================================================================
bool SimplePublisher::eventFilter(QObject *object, QEvent *event)
{
  return false;
}

//=============================================================================
std::shared_ptr<NodeData>
SimplePublisher::
outData(PortIndex)
{
  return _data->_nodeData;
}

//=============================================================================
void
SimplePublisher::
setInData(std::shared_ptr<NodeData> nodeData, PortIndex)
{
  _data->_nodeData = nodeData;

  Q_EMIT dataUpdated(0);
}
