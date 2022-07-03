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

#include "DoubleData.hpp"

//=============================================================================
DoubleData::DoubleData()
: _value(0.0)
{
  // Do nothing
}

//=============================================================================
DoubleData::DoubleData(double value)
: _value(std::move(value))
{
  // Do nothing
}

//=============================================================================
auto DoubleData::type() const -> NodeDataType
{
  return NodeDataType {"double", "Double"};
}

//=============================================================================
const double& DoubleData::value() const
{
  return _value;
}

//=============================================================================
DoubleData& DoubleData::value(double value)
{
  _value = value;
  return *this;
}

//=============================================================================
QString DoubleData::to_string() const
{
  return QString::number(_value, 'f');
}
