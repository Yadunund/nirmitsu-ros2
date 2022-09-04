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

#ifndef SRC__SWITCH_HPP
#define SRC__SWITCH_HPP

#include <QtCore/QObject>

#include <nodes/NodeDataModel>
#include <nodes/NodeData>

#include "../datatypes/BoolData.hpp"
#include "../datatypes/StringData.hpp"

#include <QCheckBox>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;

//=============================================================================
/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class Switch : public NodeDataModel
{
  Q_OBJECT

public:
  Switch();

  ~Switch() {}

  QString
  caption() const override
  { return QString("Switch"); }

  bool
  captionVisible() const override { return true; }

  static QString
  Name()
  { return QString("Switch"); }

  QString
  name() const override
  { return Switch::Name(); }

  unsigned int
  nPorts(PortType portType) const override;

  bool
  portCaptionVisible(PortType portType, PortIndex portIndex) const override
  {
    Q_UNUSED(portType); Q_UNUSED(portIndex);
    return true;
  }

  QString portCaption(PortType portType, PortIndex portIndex) const override;

  NodeDataType
  dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>
  outData(PortIndex port) override;

  void
  setInData(std::shared_ptr<NodeData> data, int) override
  {}

  QWidget*
  embeddedWidget() override { return _switch; }

protected Q_SLOTS:
  void onStateChanged(int value);

private:
  QCheckBox* _switch;
  std::shared_ptr<StringData> _string;
  std::shared_ptr<BoolData> _on;
};

#endif // SRC__SWITCH_HPP