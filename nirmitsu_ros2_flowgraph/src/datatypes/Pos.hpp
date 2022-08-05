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

#ifndef SRC__DATATYPES__POS_HPP
#define SRC__DATATYPES__POS_HPP

#include "BaseNodeData.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;

//=============================================================================
struct PosType
{
  int x;
  int y;

  PosType()
  : x(0),
    y(0)
  { }

  PosType(
    int x_,
    int y_)
  : x(x_),
    y(y_)
  {  };
};

//=============================================================================
// Implement PosType datatype
class Pos : public BaseNodeData<PosType>
{
public:
  Pos();
  Pos(PosType value);
  NodeDataType type() const override;
  const PosType& value() const final;
  Pos& value(PosType value) final;
  QString to_string() const final;

  Pos& set_x(int x);
  Pos& set_y(int y);

private:
  PosType _value;
};

#endif // SRC__DATATYPES__POS_HPP
