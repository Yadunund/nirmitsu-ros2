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

#ifndef SRC__WIDGETS__JOYSTICKWIDGET_HPP
#define SRC__WIDGETS__JOYSTICKWIDGET_HPP

#include <QWidget>
#include <QMouseEvent>
#include <QPoint>
#include <QPainter>
#include <QPaintEvent>
#include <QImage>

#include "../datatypes/Point2D.hpp"

//=============================================================================
class JoystickWidget : public QWidget
{
  Q_OBJECT
  Q_ENUMS(Priority)

public:
  JoystickWidget(QWidget* parent = 0);

signals:
  void posUpdated(Point2DType Point2Dition);

protected:
  void paintEvent(QPaintEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void sendSignal();

private:
  const int _xcenter = 114;
  const int _ycenter = 114;
  int _cx = 0;
  int _cy = 0;
  bool _mouseGrab = false;
  QImage _icon;

  int distance(int x, int y);
};

#endif // SRC__WIDGETS__JOYSTICKWIDGET_HPP