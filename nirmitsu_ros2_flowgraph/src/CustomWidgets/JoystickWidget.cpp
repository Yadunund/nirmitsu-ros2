#include "JoystickWidget.hpp"

#include <chrono>
#include <thread>

inline JoystickWidget::JoystickWidget(QWidget *parent) : QWidget(parent)
{

    setFixedWidth(250);
    setFixedHeight(250);
    setAttribute(Qt::WA_TranslucentBackground);
}