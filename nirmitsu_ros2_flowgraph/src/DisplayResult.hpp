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

#ifndef SRC__DISPLAYRESULT_HPP
#define SRC__DISPLAYRESULT_HPP

#include <QtCore/QObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>
#include <nodes/NodeData>

#include <vector>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;

//=============================================================================
class TextData : public NodeData
{
public:

  TextData() {}

  TextData(QString const &text)
    : _text(text)
  {}

  NodeDataType type() const override
  { return NodeDataType {"result", "Result"}; }

  QString text() const { return _text; }

private:

  QString _text;
};

//=============================================================================
/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class DisplayResult : public NodeDataModel
{
  Q_OBJECT

public:
  DisplayResult();

  ~DisplayResult() {}

  QString
  caption() const override
  { return QString("Result"); }

  bool
  captionVisible() const override { return false; }

  static QString
  Name()
  { return QString("DisplayResult"); }

  QString
  name() const override
  { return DisplayResult::Name(); }

  unsigned int
  nPorts(PortType portType) const override;

  NodeDataType
  dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>
  outData(PortIndex port) override;

  ConnectionPolicy
  portInConnectionPolicy(PortIndex) const override
  {
    return ConnectionPolicy::Many;
  }

  void
  setInData(std::shared_ptr<NodeData> data, int) override
  { }

  void
  setInData(std::shared_ptr<NodeData> data, int, const QUuid& connectionId) override
  {
    auto textData = std::dynamic_pointer_cast<TextData>(data);

    if (!textData)
      return;

    auto it = std::find_if(inputTexts.begin(), inputTexts.end(),
        [this, &connectionId](const auto& e)
        {
            return e.first == connectionId;
        });
    if (textData)
    {
      if (it == inputTexts.end())
        inputTexts.emplace_back(connectionId, textData->text());
      else
        it->second = textData->text();
    }
    else
    {
      inputTexts.erase(it);
    }

    QStringList textList;
    for (auto&& entry : inputTexts) textList.push_back(entry.second);

    _label->setText(QStringLiteral("%1 inputs: %2")
        .arg(textList.size())
        .arg(textList.join(QStringLiteral(", "))));
    _label->adjustSize();
  }

  QWidget *
  embeddedWidget() override { return _label; }

private:

  QLabel * _label;
  std::vector<std::pair<QUuid, QString>> inputTexts;
};

#endif // SRC__DISPLAYRESULT_HPP