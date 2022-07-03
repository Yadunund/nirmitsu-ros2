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

#include "Switch.hpp"

//=============================================================================
Switch::Switch()
: _switch(new QCheckBox("On")),
  _string(std::make_shared<StringData>()),
  _on(std::make_shared<BoolData>(0))
{
  connect(
    _switch,
    &QCheckBox::stateChanged,
    this,
    &Switch::onStateChanged
  );
  // Initialize to 0
  onStateChanged(0);
}

//=============================================================================
unsigned int Switch::nPorts(PortType portType) const
{
  if (portType == PortType::In)
    return 0;
  else if (portType == PortType::Out)
    return 2;
  else
    return 0;
}

//=============================================================================
NodeDataType Switch::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
    return NodeDataType();

  if (portIndex == 0)
    return _string->type();
  else if (portIndex == 1)
    return _on->type();
  else
    return NodeDataType();
}

//=============================================================================
std::shared_ptr<NodeData> Switch::outData(PortIndex portIndex)
{
  if (portIndex == 0)
    return _string;
  else if (portIndex == 1)
    return _on;
  else
    return nullptr;
}

//=============================================================================
void Switch::onStateChanged(int value)
{
  _on->value(value);
  _string->value(
    QStringLiteral("State: %1").arg(_on->to_string()));
  Q_EMIT dataUpdated(0);
  Q_EMIT dataUpdated(1);
}
