#include "SimplePublisher.hpp"

#include <QtCore/QEvent>
#include <QtCore/QDir>

#include <QtWidgets/QFileDialog>

SimplePublisher::SimplePublisher()
  : _label(new QLabel("Image will appear here"))
{
  _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

  QFont f = _label->font();
  f.setBold(true);
  f.setItalic(true);

  _label->setFont(f);

  _label->setFixedSize(200, 200);

  _label->installEventFilter(this);

  _period = std::chrono::seconds(1);

  _node = std::make_shared<rclcpp::Node>("simple_publisher_node");
  _pub = _node->create_publisher<std_msgs::msg::String>(
    "/chatter",
    rclcpp::SystemDefaultsQoS()
  );

  _pub_thread = std::thread(
    [&]()
    {
      while (rclcpp::ok())
      {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello from flow component";
        _pub->publish(std::move(msg));
        std::this_thread::sleep_for(_period);
      }
    });

  _spin_thread = std::thread(
    [&]()
    {
      rclcpp::spin(_node);
    }
  );
}

SimplePublisher::~SimplePublisher()
{
  if(_spin_thread.joinable())
    _spin_thread.join();
  // rclcpp::shutdown();
}

unsigned int
SimplePublisher::
nPorts(PortType portType) const
{
  unsigned int result = 1;

  switch (portType)
  {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 1;

    default:
      break;
  }

  return result;
}


bool SimplePublisher::eventFilter(QObject *object, QEvent *event)
{
  return false;
}


std::shared_ptr<NodeData>
SimplePublisher::
outData(PortIndex)
{
  return _nodeData;
}


void
SimplePublisher::
setInData(std::shared_ptr<NodeData> nodeData, PortIndex)
{
  _nodeData = nodeData;

  Q_EMIT dataUpdated(0);
}
