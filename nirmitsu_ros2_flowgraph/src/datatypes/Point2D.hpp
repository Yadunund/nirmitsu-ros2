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

#ifndef SRC__DATATYPES__POINT2D_HPP
#define SRC__DATATYPES__POINT2D_HPP

#include "BaseNodeData.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;

//=============================================================================
struct Point2DType
{
  int x;
  int y;

  Point2DType()
  : x(0),
    y(0)
  {}

  Point2DType(
    int x_,
    int y_)
  : x(x_),
    y(y_)
  {}
};

//=============================================================================
// Implement Point2DType datatype
class Point2D : public BaseNodeData<Point2DType>
{
public:
  Point2D();
  Point2D(Point2DType value);
  NodeDataType type() const override;
  const Point2DType& value() const final;
  Point2D& value(Point2DType value) final;
  QString to_string() const final;

  Point2D& set_x(int x);
  Point2D& set_y(int y);

private:
  Point2DType _value;
};

#endif // SRC__DATATYPES__POINT2D_HPP
