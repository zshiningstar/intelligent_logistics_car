#pragma execution_character_set("utf-8")

#include "imageswitch.hpp"
#include "qpainter.h"
#include "qdebug.h"

ImageSwitch::ImageSwitch(QWidget *parent) : QWidget(parent)
{
    isChecked = false;
    buttonStyle = ButtonStyle_2;
    useClick = true;

    imgOffFile = ":/button/gray_led";
    imgOnFile = ":/button/green_led";
    imgFile = imgOffFile;
}

void ImageSwitch::mousePressEvent(QMouseEvent *event)
{
    //qDebug() << "mousePressEvent " << useClick << "\t" << event->button();
    if(!useClick)
    {
        QWidget::mousePressEvent(event);
        return;
    }

    if(event->button() == Qt::LeftButton)
    {
        isChecked = !isChecked;
        imgFile = isChecked ? imgOnFile : imgOffFile;

        this->update();
    }
    this->QWidget::mousePressEvent(event);
}

void ImageSwitch::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHints(QPainter::SmoothPixmapTransform);
    QImage img(imgFile);
    //等比、平滑缩放
    //将图片尺寸缩放到小于控件尺寸，防止绘制不全
    img = img.scaled(this->size()*0.9, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    //按照比例自动居中绘制
    int pixX = rect().center().x() - img.width() / 2;
    int pixY = rect().center().y() - img.height() / 2;
    QPoint point(pixX, pixY);
    painter.drawImage(point, img);
    this->QWidget::paintEvent(event);
}

bool ImageSwitch::getChecked() const
{
    return isChecked;
}

ImageSwitch::ButtonStyle ImageSwitch::getButtonStyle() const
{
    return this->buttonStyle;
}

/*
QSize ImageSwitch::sizeHint() const
{
    return QSize(87, 40);
}

QSize ImageSwitch::minimumSizeHint() const
{
    return QSize(87, 40);
}
*/
void ImageSwitch::setChecked(bool isChecked)
{
    if (this->isChecked != isChecked)
    {
        this->isChecked = isChecked;
        imgFile = isChecked ? imgOnFile : imgOffFile;
        this->update();
    }
}

void ImageSwitch::setButtonStyle(const ImageSwitch::ButtonStyle &buttonStyle)
{
    if (this->buttonStyle != buttonStyle)
    {
        this->buttonStyle = buttonStyle;

        if (buttonStyle == ButtonStyle_1) {
            imgOffFile = ":/image/btncheckoff1.png";
            imgOnFile = ":/image/btncheckon1.png";
            this->resize(87, 28);
        } else if (buttonStyle == ButtonStyle_2) {
            imgOffFile = ":/image/btncheckoff2.png";
            imgOnFile = ":/image/btncheckon2.png";
            this->resize(87, 28);
        } else if (buttonStyle == ButtonStyle_3) {
            imgOffFile = ":/image/btncheckoff3.png";
            imgOnFile = ":/image/btncheckon3.png";
            this->resize(96, 38);
        } else if (buttonStyle == ButtonStyle_4) {
            imgOffFile = ":/button/gray_led";
            imgOnFile = ":/button/green_led";
            this->resize(30,30);
        }

        imgFile = isChecked ? imgOnFile : imgOffFile;
        setChecked(isChecked);
        this->update();
        updateGeometry();
    }
}
