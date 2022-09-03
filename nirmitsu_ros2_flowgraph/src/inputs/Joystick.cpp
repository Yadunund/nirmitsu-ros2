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

#include "Joystick.hpp"

//=============================================================================
Joystick::Joystick()
: _joystick(new JoystickWidget()),
  _position(std::make_shared<Point2D>(Point2DType(0, 0))),
  _string(std::make_shared<StringData>())
{
  connect(
    _joystick,
    &JoystickWidget::posUpdated,
    this,
    &Joystick::onMove
  );
  //Initialize to 0, 0
  onMove(Point2DType(
      0, 0
  ));

}

//=============================================================================
unsigned int Joystick::nPorts(PortType portType) const
{
  if (portType == PortType::In)
    return 0;
  else if (portType == PortType::Out)
    return 2;
  else
    return 0;
}

//=============================================================================
QString Joystick::portCaption(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
  {
    return QString();
  }
  else if (portType == PortType::Out)
  {
    if (portIndex == 0)
      return QStringLiteral("Display");
    else
      return QStringLiteral("Joystick");
  }
  else
  {
    return QString();
  }
}

//=============================================================================
NodeDataType Joystick::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
    return NodeDataType();

  if (portIndex == 0)
    return _string->type();
  else if (portIndex == 1)
    return _position->type();
  else
    return NodeDataType();
}

//=============================================================================
std::shared_ptr<NodeData> Joystick::outData(PortIndex portIndex)
{
  if (portIndex == 0)
    return _string;
  else if (portIndex == 1)
    return _position;
  else
    return nullptr;
}
//=============================================================================
void Joystick::onMove(Point2DType position)
{
  _position->value(position);
  _string->value(_position->to_string());
  Q_EMIT dataUpdated(0);
  Q_EMIT dataUpdated(1);
}
