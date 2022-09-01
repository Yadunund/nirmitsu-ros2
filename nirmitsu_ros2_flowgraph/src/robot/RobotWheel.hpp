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

#ifndef SRC__RobotWheelWHEEL_HPP
#define SRC__RobotWheelWHEEL_HPP

#include <QtCore/QObject>
#include <QtCore/QEvent>

#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>
#include <nodes/NodeData>

#include "../datatypes/StringData.hpp"
#include "../datatypes/IntegerData.hpp"
#include "../datatypes/WheelData.hpp"
#include "../datatypes/BoolData.hpp"

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;

///=============================================================================
// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class RobotWheel : public NodeDataModel
{
  Q_OBJECT

public:
  RobotWheel();

  QString
  caption() const override
  { return QString("Wheel"); }

  bool
  captionVisible() const override { return true; }

  static QString
  Name()
  { return QString("Wheel"); }

  QString
  name() const override
  { return RobotWheel::name(); }

  unsigned int
  nPorts(PortType portType) const override;

  bool
  portCaptionVisible(PortType portType, PortIndex portIndex) const override
  {
    Q_UNUSED(portType); Q_UNUSED(portIndex);
    return true;
  }

  QString portCaption(PortType portType, PortIndex portIndex) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData> outData(PortIndex portIndex) override;

  void setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) override;

  QWidget* embeddedWidget() override;

  bool resizable() const override { return true; }

private:
  QLabel* _label;
  std::shared_ptr<StringData> _string_data;
  std::shared_ptr<StringData> _name_data;
  std::shared_ptr<IntegerData> _speed_data;
  std::shared_ptr<BoolData> _reverse_data;
  std::shared_ptr<WheelData> _wheel_data;
};

#endif // SRC__RobotWheelWHEEL_HPP
