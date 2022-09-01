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

#include "Point2D.hpp"
#include <iostream>

//=============================================================================
Point2D::Point2D()
: _value(Point2DType())
{
  // Do nothing
}

//=============================================================================
Point2D::Point2D(Point2DType value)
: _value(std::move(value))
{
  // Do nothing
}

//=============================================================================
auto Point2D::type() const -> NodeDataType
{
  return NodeDataType {"Point2D", "Point2D"};
}

//=============================================================================
const Point2DType& Point2D::value() const
{
  return _value;
}

//=============================================================================
Point2D& Point2D::value(Point2DType value)
{
  _value = std::move(value);
  return *this;
}

//=============================================================================
QString Point2D::to_string() const
{
  return QStringLiteral("X: %1\nY: %2")
    .arg(QString::number(_value.x))
    .arg(QString::number(_value.y));
}

//=============================================================================
Point2D& Point2D::set_x(int x)
{
  _value.x = x;
  return *this;
}

//=============================================================================
Point2D& Point2D::set_y(int y)
{
  _value.y = y;
  return *this;
}