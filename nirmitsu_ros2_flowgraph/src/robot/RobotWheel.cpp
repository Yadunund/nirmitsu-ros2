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
: _label(new QLabel()),
  _string_data(std::make_shared<StringData>()),
  _speed_data(nullptr),
  _name_data(nullptr),
  _reverse_data(std::make_shared<BoolData>(false)),
  _wheel_data(nullptr)
{
  _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
}

//=============================================================================
unsigned int RobotWheel::nPorts(PortType portType) const
{
  if (portType == PortType::In)
    return 3;
  else if (portType == PortType::Out)
    return 2;
  else
    return 0;
}

//=============================================================================
QString RobotWheel::portCaption(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
    if (portIndex == 0)
      return QStringLiteral("Name");
    else if (portIndex == 1)
      return QStringLiteral("Speed");
    else if (portIndex == 2)
      return QStringLiteral("Reverse");
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
    // Name
    if (portIndex == 0)
    {
      return StringData().type();
    }
    // Slider
    else if (portIndex == 1)
    {
      return IntegerData().type();
    }
    // Reverse
    else if (portIndex == 2)
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
    if (portIndex == 0)
    {
      return _string_data->type();
    }
    else if (portIndex == 1)
    {
      return WheelData().type();
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
  // Name
  if (portIndex == 0)
  {
    auto name = std::dynamic_pointer_cast<StringData>(data);
    if (name == nullptr || name->value().isEmpty())
      return;
    _name_data = std::move(name);
    _label->setText(QStringLiteral("%1 Wheel").arg(_name_data->value()));
    _label->adjustSize();
    if (_wheel_data != nullptr)
      _wheel_data->set_name(_name_data->value());
  }
  // Speed
  else if (portIndex == 1)
  {
    auto speed_data = std::dynamic_pointer_cast<IntegerData>(data);
    if (speed_data == nullptr)
      return;
    _speed_data = std::move(speed_data);
    if (_wheel_data != nullptr)
      _wheel_data->set_speed(_speed_data->value());
  }
  // Reverse
  else if (portIndex == 2)
  {
    auto reverse_data = std::dynamic_pointer_cast<BoolData>(data);
    if (reverse_data == nullptr)
      return;
    _reverse_data = std::move(reverse_data);
    if (_wheel_data != nullptr)
      _wheel_data->set_reverse(_reverse_data->value());
  }
  else
  {
    return;
  }

  // Create or update wheel data if both name and speed are provided.
  if (_wheel_data == nullptr)
  {
    if (_name_data != nullptr && _speed_data != nullptr)
    {
      _wheel_data = std::make_shared<WheelData>(WheelDataType(
            _name_data->value(),
            _speed_data->value(),
            _reverse_data->value()
      ));
      _string_data->value(_wheel_data->to_string());
      Q_EMIT dataUpdated(0);
      Q_EMIT dataUpdated(1);
    }
  }
  else
  {
    _string_data->value(_wheel_data->to_string());
    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
  }
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
