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

#include "DisplayResult.hpp"

//=============================================================================
DisplayResult::
DisplayResult()
  : _label(new QLabel("Result"))
{
  _label->setMargin(3);
}

//=============================================================================
unsigned int
DisplayResult::
nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 0;

    default:
      break;
  }

  return result;
}

//=============================================================================
NodeDataType
DisplayResult::
dataType(PortType, PortIndex) const
{
  return TextData().type();
}

//=============================================================================
std::shared_ptr<NodeData>
DisplayResult::
outData(PortIndex)
{
  std::shared_ptr<NodeData> ptr;
  return ptr;
}
