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

#include "WheelData.hpp"

//=============================================================================
WheelData::WheelData()
: _value(WheelDataType())
{
  // Do nothing
}

//=============================================================================
WheelData::WheelData(WheelDataType value)
: _value(std::move(value))
{
  // Do nothing
}

//=============================================================================
auto WheelData::type() const -> NodeDataType
{
  return NodeDataType {"wheel", "Wheel"};
}

//=============================================================================
const WheelDataType& WheelData::value() const
{
  return _value;
}

//=============================================================================
WheelData& WheelData::value(WheelDataType value)
{
  _value = std::move(value);
  return *this;
}

//=============================================================================
QString WheelData::to_string() const
{
  return QStringLiteral("Name: %1\nSpeed: %2\nReverse: %3")
    .arg(_value.name)
    .arg(QString::number(_value.speed)
      .arg(_value.reverse ? QStringLiteral("True") : QStringLiteral("False"))
    );
}

//=============================================================================
WheelData& WheelData::set_speed(int speed)
{
  _value.speed = speed;
  return *this;
}

//=============================================================================
WheelData& WheelData::set_name(QString name)
{
  _value.name = std::move(name);
  return *this;
}

//=============================================================================
WheelData& WheelData::set_reverse(bool reverse)
{
  _value.reverse = reverse;
  return *this;
}