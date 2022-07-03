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

#include "BoolData.hpp"

//=============================================================================
BoolData::BoolData()
: _value(bool())
{
  // Do nothing
}

//=============================================================================
BoolData::BoolData(bool value)
: _value(std::move(value))
{
  // Do nothing
}

//=============================================================================
auto BoolData::type() const -> NodeDataType
{
  return NodeDataType {"bool", "Boolean"};
}

//=============================================================================
bool BoolData::value() const
{
  return _value;
}

//=============================================================================
BoolData& BoolData::value(bool value)
{
  _value = value;
  return *this;
}

//=============================================================================
bool BoolData::to_string() const
{
  return _value;
}
