#ifndef SRC__JOYSTICKWIDGET_HPP
#define SRC__JOYSTICKWIDGET_HPP

#include <qwidget.h>
#include"../datatypes/Pos.hpp"
#include <QtGui>
#include <math.h>

class JoystickWidget : public QWidget
{
    Q_OBJECT
    Q_ENUMS(Priority)

public:
    const int xcenter = 114;
    const int ycenter = 114;
    int cx = 0;
    int cy = 0;
    bool mouseGrab = false;

    inline int distance(int x, int y);
    inline JoystickWidget(QWidget *parent = 0);

signals:
    void posUpdated(PosType position);

protected:
    inline void paintEvent(QPaintEvent *event);
    inline void mousePressEvent(QMouseEvent * event);
    inline void mouseReleaseEvent(QMouseEvent * event);
    inline void mouseMoveEvent(QMouseEvent * event);
    inline void sendSignal();

};

inline void JoystickWidget::paintEvent(QPaintEvent*) {

    QPainter painter(this);

    QRectF rectangle(15, 30, 200, 200);

    painter.setPen(QPen(Qt::white, 8, Qt::SolidLine, Qt::RoundCap));
    painter.drawArc(rectangle, 0*16, 360*16);

    painter.drawImage(QRect(75+cx, 91+cy, 80, 80), QImage("/home/dhruft/ws_nirmitsu/src/nirmitsu-ros2/nirmitsu_ros2_flowgraph/src/CustomWidgets/joystick.png"));

}

inline void JoystickWidget::mousePressEvent(QMouseEvent *event)
{
    if (distance(event->x(), event->y())) {
        cx = event->x()-xcenter;
        cy = event->y()-ycenter;
        mouseGrab = true;
        sendSignal();
    }
}

inline void JoystickWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (mouseGrab) {
        cx = 0;
        cy = 0;
        mouseGrab = false;
        sendSignal();
    }

}

inline void JoystickWidget::mouseMoveEvent(QMouseEvent * event) {
    
    if (mouseGrab) {
        if (distance(event->x(), event->y())) {
            cx = event->x()-xcenter;
            cy = event->y()-ycenter;
        } else {
            double fulldist = sqrt( pow((event->x()-xcenter),2) + pow((event->y()-ycenter),2) );
            cx = (65/fulldist) * (event->x() - xcenter);
            cy = (65/fulldist) * (event->y() - ycenter);
            
        }
        
        sendSignal();
    }

}

inline void JoystickWidget::sendSignal() {
    update();
    emit posUpdated(PosType(
        round(100*(cx/65.0)), round(100*(cy/65.0))
    ));
}

inline int JoystickWidget::distance(int x, int y) {
    if (sqrt( pow((x-xcenter),2) + pow((y-ycenter),2) ) <= 65) {
        return true;
    }
    return false;
}

#endif // SRC__JOYSTICKWIDGET_HPP