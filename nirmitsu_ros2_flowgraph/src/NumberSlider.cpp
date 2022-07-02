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

#include "NumberSlider.hpp"

//=============================================================================
NumberSlider::NumberSlider()
: _slider(new QSlider(Qt::Horizontal)),
  _string(std::make_shared<StringData>()),
  _number(std::make_shared<IntegerData>(0))
{
  // QSlider can only accept integer values
  _slider->setMinimum(0);
  _slider->setMaximum(100);
  _slider->setSingleStep(1);
  connect(
    _slider,
    &QSlider::valueChanged,
    this,
    &NumberSlider::onSliderUpdated
  );
  // Initialize to 0
  onSliderUpdated(0);
}

//=============================================================================
unsigned int NumberSlider::nPorts(PortType portType) const
{
  if (portType == PortType::In)
    return 0;
  else if (portType == PortType::Out)
    return 2;
  else
    return 0;
}

//=============================================================================
NodeDataType NumberSlider::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
    return NodeDataType();
  
  if (portIndex == 0)
    return _string->type();
  else if (portIndex == 1)
    return _number->type();
  else
    return NodeDataType();
}

//=============================================================================
std::shared_ptr<NodeData> NumberSlider::outData(PortIndex portIndex)
{
  switch (portIndex)
  {
    case 0:
      return _string;
    case 1:
      return _number;
    default:
      break;
  }

  return nullptr;
}

//=============================================================================
void NumberSlider::onSliderUpdated(int value)
{
  _number->value(value);
  _string->value(
    QStringLiteral("Slider value: %1").arg(QString::number(value)));
  Q_EMIT dataUpdated(0);
  Q_EMIT dataUpdated(1);
}
