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

#include "RobotWheel.hpp"

//=============================================================================
RobotWheel::RobotWheel()
: _label(new QLabel("Left")),
  _string_data(std::make_shared<StringData>()),
  _wheel_data(std::make_shared<WheelData>()),
  _slider_data(std::make_shared<IntegerData>(0))
{
  _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
  _wheel_data->set_name(_label->text());
  _wheel_data->set_on(true);
  _wheel_data->set_speed(0);

  Q_EMIT dataUpdated(0);
  Q_EMIT dataUpdated(1);
}

//=============================================================================
unsigned int RobotWheel::nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 2;
      break;
    default:
      break;
  }

  return result;
}

//=============================================================================
QString RobotWheel::portCaption(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
      if (portIndex == 0)
        return QStringLiteral("Speed");
      else
        return QString();
  }
  else if (portType == PortType::Out)
  {
    if (portIndex == 0)
      return QStringLiteral("Display");
    else if (portIndex == 1)
      return QStringLiteral("Wheel");
    else
      return QString();
  }
  else
  {
    return QString();
  }
}

//=============================================================================
NodeDataType RobotWheel::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
    // Slider
    if (portIndex == 0)
    {
      return _slider_data->type();
    }
    else if (portIndex == 1)
    {
      return _wheel_data->type();
    }
    else
    {
      return NodeDataType();
    }
  }
  else if (portType == PortType::Out)
  {
    if (portIndex == 0)
    {
      return _string_data->type();
    }
    else if (portIndex == 1)
    {
      return _wheel_data->type();
    }
    else
    {
      return NodeDataType();
    }
  }
  else
  {
    return NodeDataType();
  }
}

//=============================================================================
void RobotWheel::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
{
  // Slider
  if (portIndex == 0)
  {
    auto speed_data = std::dynamic_pointer_cast<IntegerData>(data);
    if (speed_data == nullptr)
      return;
    _wheel_data->set_speed(speed_data->value());
  }
  else
  {
    return;
  }
  _string_data->value(_wheel_data->to_string());
  Q_EMIT dataUpdated(0);
  Q_EMIT dataUpdated(1);
}

//=============================================================================
std::shared_ptr<NodeData> RobotWheel::outData(PortIndex portIndex)
{
  if (portIndex == 0)
  {
    return _string_data;
  }
  else if (portIndex == 1)
  {
    return _wheel_data;
  }
  else
  {
    return nullptr;
  }
}

//=============================================================================
QWidget* RobotWheel::embeddedWidget()
{
  return _label;
}