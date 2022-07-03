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

#ifndef SRC__DATATYPES__BOOLDATA_HPP
#define SRC__DATATYPES__BOOLDATA_HPP

#include "BaseNodeData.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;

//=============================================================================
// Implement bool datatype
class BoolData : public BaseNodeData<bool>
{
public:
  BoolData();
  BoolData(bool value);
  NodeDataType type() const override;
  const bool& value() const final;
  BoolData& value(bool value) final;
  QString to_string() const final;

private:
  bool _value;
};

#endif // SRC__DATATYPES__BOOLDATA_HPP
