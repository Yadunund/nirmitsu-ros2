#ifndef SRC__SIMPLEPUBLISHER_HPP
#define SRC__SIMPLEPUBLISHER_HPP

#include <iostream>

#include <QtCore/QObject>
#include <QtWidgets/QLabel>

#include <nodes/DataModelRegistry>
#include <nodes/NodeDataModel>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <memory>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

class MyNodeData : public NodeData
{
public:

  NodeDataType
  type() const override
  { return NodeDataType {"MyNodeData", "My Node Data"}; }
};

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class SimplePublisher : public NodeDataModel
{
  Q_OBJECT

public:
  SimplePublisher();

  ~SimplePublisher();

public:

  QString
  caption() const override
  { return QString("Simple publisher"); }

  QString
  name() const override
  { return QString("SimplePublisher"); }

public:

  virtual QString
  modelName() const
  { return QString("Resulting Image"); }

  unsigned int
  nPorts(PortType portType) const override;

  NodeDataType
  dataType(PortType, PortIndex) const override
  {
    return MyNodeData().type();
  }

  std::shared_ptr<NodeData>
  outData(PortIndex port) override;

  void
  setInData(std::shared_ptr<NodeData> nodeData, PortIndex port) override;

  QWidget *
  embeddedWidget() override { return _label; }

  bool
  resizable() const override { return true; }

protected:

  bool
  eventFilter(QObject *object, QEvent *event) override;

private:
  QLabel * _label;

  std::shared_ptr<NodeData> _nodeData;

  std::thread _spin_thread;
  std::thread _pub_thread;
  std::chrono::nanoseconds _period;
  rclcpp::Node::SharedPtr _node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
  rclcpp::TimerBase::SharedPtr _timer;

};

#endif // SRC__SIMPLEPUBLISHER_HPP