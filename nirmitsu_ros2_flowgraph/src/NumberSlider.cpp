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
}

//=============================================================================
unsigned int NumberSlider::nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 0;
      break;

    case PortType::Out:
      result = 1;

    default:
      break;
  }

  return result;
}

//=============================================================================
NodeDataType NumberSlider::dataType(PortType portType, PortIndex portIndex) const
{
  return _number->type();
}

//=============================================================================
std::shared_ptr<NodeData> NumberSlider::outData(PortIndex port)
{
  return _number;
}

//=============================================================================
void NumberSlider::onSliderUpdated(int value)
{
  _number->value(value);
  Q_EMIT dataUpdated(0);
}
