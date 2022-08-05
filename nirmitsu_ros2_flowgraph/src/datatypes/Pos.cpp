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

#include "Pos.hpp"
#include <iostream>

//=============================================================================
Pos::Pos()
: _value(PosType())
{
  // Do nothing
}

//=============================================================================
Pos::Pos(PosType value)
: _value(std::move(value))
{
  // Do nothing
}

//=============================================================================
auto Pos::type() const -> NodeDataType
{
  return NodeDataType {"position", "Position"};
}

//=============================================================================
const PosType& Pos::value() const
{
  return _value;
}

//=============================================================================
Pos& Pos::value(PosType value)
{
  _value = std::move(value);
  return *this;
}

//=============================================================================
QString Pos::to_string() const
{
  return QStringLiteral("X Pos: %1\nY Pos: %2")
    .arg(QString::number(_value.x))
    .arg(QString::number(_value.y));
}

//=============================================================================
Pos& Pos::set_x(int x)
{
  _value.x = x;
  return *this;
}

//=============================================================================
Pos& Pos::set_y(int y)
{
  _value.y = y;
  return *this;
}