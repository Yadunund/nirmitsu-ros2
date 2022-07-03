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

#include "TextBox.hpp"

//=============================================================================
TextBox::TextBox()
: _text_box(new QTextEdit()),
  _string(std::make_shared<StringData>())
{
  connect(
    _text_box,
    &QTextEdit::textChanged,
    this,
    &TextBox::onTextChanged
  );

}

//=============================================================================
unsigned int TextBox::nPorts(PortType portType) const
{
  if (portType == PortType::In)
    return 0;
  else if (portType == PortType::Out)
    return 1;
  else
    return 0;
}

//=============================================================================
QString TextBox::portCaption(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::Out && portIndex == 0)
    return QStringLiteral("Text");

  return QString();
}

//=============================================================================
NodeDataType TextBox::dataType(PortType portType, PortIndex portIndex) const
{
  if (portType == PortType::In)
    return NodeDataType();

  if (portIndex == 0)
    return _string->type();
  else
    return NodeDataType();
}

//=============================================================================
std::shared_ptr<NodeData> TextBox::outData(PortIndex portIndex)
{
  if (portIndex == 0)
  {
    return _string;
  }

  return nullptr;
}

//=============================================================================
void TextBox::onTextChanged()
{
  _string->value(_text_box->toPlainText());
  const QSize& size = _text_box->document()->size().toSize();
  _text_box->setFixedHeight(size.height() + 3);
  Q_EMIT dataUpdated(0);
}
