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

#include "JoystickWidget.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <iostream>

//=============================================================================
JoystickWidget::JoystickWidget(QWidget* parent)
: QWidget(parent)
{
  setFixedWidth(250);
  setFixedHeight(250);
  setAttribute(Qt::WA_TranslucentBackground);
  const std::string& icon_path = ament_index_cpp::get_package_share_directory(
    "nirmitsu_ros2_flowgraph") + "/icons/joystick.png";
  std::cout << "Loading joystick icon from: " << icon_path << std::endl;
  _icon = QImage(QString::fromStdString(icon_path));
}

//=============================================================================
void JoystickWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  QRectF rectangle(15, 30, 200, 200);

  painter.setPen(QPen(Qt::white, 8, Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(rectangle, 0*16, 360*16);
  painter.drawImage(
    QRect(75+_cx, 91+_cy, 80, 80),
    _icon);
}

//=============================================================================
void JoystickWidget::mousePressEvent(QMouseEvent* event)
{
  if (distance(event->pos().x(), event->pos().y()))
  {
    _cx = event->x()- _xcenter;
    _cy = event->y()- _ycenter;
    _mouseGrab = true;
    sendSignal();
  }
}

//=============================================================================
void JoystickWidget::mouseReleaseEvent(QMouseEvent*)
{
  if (_mouseGrab)
  {
    _cx = 0;
    _cy = 0;
    _mouseGrab = false;
    sendSignal();
  }
}

//=============================================================================
void JoystickWidget::mouseMoveEvent(QMouseEvent* event)
{

  if (_mouseGrab)
  {
    if (distance(event->x(), event->y()))
    {
      _cx = event->x() - _xcenter;
      _cy = event->y() - _ycenter;
    }
    else
    {
      double fulldist =
        std::sqrt(std::pow((event->x() - _xcenter),
          2) + std::pow((event->y() - _ycenter), 2));
      _cx = (65/fulldist) * (event->x() - _xcenter);
      _cy = (65/fulldist) * (event->y() - _ycenter);
    }
    sendSignal();
  }

}

//=============================================================================
void JoystickWidget::sendSignal()
{
  update();
  emit posUpdated(Point2DType(
      round(100*(_cx/65.0)), -1 * round(100*(_cy/65.0))
  ));
}

//=============================================================================
int JoystickWidget::distance(int x, int y)
{
  if (std::sqrt(std::pow((x-_xcenter), 2) + std::pow((y-_ycenter), 2) ) <= 65)
  {
    return true;
  }
  return false;
}
