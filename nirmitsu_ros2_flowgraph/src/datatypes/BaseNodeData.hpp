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

#ifndef SRC__DATATYPES__BASENODEDAATA_HPP
#define SRC__DATATYPES__BASENODEDAATA_HPP

#include <nodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

//=============================================================================
// This is a pure abstract base class that other NodeData classes in nirmitsu
// should inherit. It adds a convenience function to output the data structure
// in a QString format. NodeDataModels in nirmitsu should attempt to dynamic
// cast to this data type.
template <typename T>
class BaseNodeData : public NodeData
{
public:
  // Get the value
  virtual T value() const = 0;

  // Set the value
  virtual BaseNodeData& value(T) = 0;

  // Get a string representation of the value
  virtual QString to_string() const = 0;

  ~BaseNodeData() = default;
};

#endif // SRC__DATATYPES__BASENODEDAATA_HPP