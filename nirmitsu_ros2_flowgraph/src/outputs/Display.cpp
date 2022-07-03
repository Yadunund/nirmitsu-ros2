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

#include"../datatypes/StringData.hpp"

#include "Display.hpp"

//=============================================================================
Display::Display()
: _text_box(new QTextEdit())
{
  // _text_box->setFrameRect(QRect(0, 0, 10, 10));
}

//=============================================================================
unsigned int Display::nPorts(PortType portType) const
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
NodeDataType Display::dataType(PortType, PortIndex) const
{
  return StringData().type();
}

//=============================================================================
std::shared_ptr<NodeData> Display::outData(PortIndex)
{
  std::shared_ptr<NodeData> ptr;
  return ptr;
}


//=============================================================================
void Display::setInData(std::shared_ptr<NodeData> data, int)
{
  auto string_data = std::dynamic_pointer_cast<StringData>(data);

  if (string_data)
  {
    _text_box->setText(string_data->value());
    const QSize& size = _text_box->document()->size().toSize();
    _text_box->setFixedHeight(size.height() + 3);
  }
  else
  {
    _text_box->clear();
  }

  _text_box->adjustSize();
}