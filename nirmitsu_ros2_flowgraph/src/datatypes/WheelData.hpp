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

#ifndef SRC__DATATYPES__WHEELDATA_HPP
#define SRC__DATATYPES__WHEELDATA_HPP

#include "BaseNodeData.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;

//=============================================================================
struct WheelDataType
{
  QString name;
  int speed;
  bool reverse;

  WheelDataType()
  : name(QString()),
    speed(0),
    reverse(false)
  {}

  WheelDataType(
    QString name_,
    int speed_,
    bool reverse_)
  : name(std::move(name_)),
    speed(speed_),
    reverse(reverse_)
  {}
};

//=============================================================================
// Implement WheelDataType datatype
class WheelData : public BaseNodeData<WheelDataType>
{
public:
  WheelData();
  WheelData(WheelDataType value);
  NodeDataType type() const override;
  const WheelDataType& value() const final;
  WheelData& value(WheelDataType value) final;
  QString to_string() const final;

  WheelData& set_name(QString name);
  WheelData& set_speed(int speed);
  WheelData& set_reverse(bool reverse);

private:
  WheelDataType _value;
};

#endif // SRC__DATATYPES__WHEELDATA_HPP
