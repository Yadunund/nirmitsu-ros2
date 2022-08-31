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

#ifndef SRC__DISPLAY_HPP
#define SRC__DISPLAY_HPP

#include <QtCore/QObject>
#include <QTextEdit>

#include <nodes/NodeDataModel>
#include <nodes/NodeData>

#include <vector>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;

//=============================================================================
/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class Display : public NodeDataModel
{
  Q_OBJECT

public:
  Display();

  ~Display() {}

  QString
  caption() const override
  { return QString("Display"); }

  bool
  captionVisible() const override { return true; }

  static QString
  Name()
  { return QString("Display"); }

  QString
  name() const override
  { return Display::Name(); }

  unsigned int
  nPorts(PortType portType) const override;

  NodeDataType
  dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>
  outData(PortIndex port) override;

  void
  setInData(std::shared_ptr<NodeData> data, int) override;

  QWidget*
  embeddedWidget() override { return _text_box; }

private:

  QTextEdit* _text_box;
};

#endif // SRC__DISPLAY_HPP