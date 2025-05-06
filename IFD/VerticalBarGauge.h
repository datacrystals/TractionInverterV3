#pragma once
#include <QWidget>
#include <QPainter>
#include <QLabel>
#include <QVBoxLayout>

class VerticalBarGauge : public QWidget {
    Q_OBJECT
public:
    explicit VerticalBarGauge(QWidget *parent = nullptr)
        : QWidget(parent), minValue(0), maxValue(100), currentValue(0),
          barWidth(30), titleLabel(new QLabel), valueLabel(new QLabel) {
            
        QVBoxLayout *layout = new QVBoxLayout(this);
        layout->addWidget(titleLabel);
        layout->addStretch();
        layout->addWidget(valueLabel);
        
        titleLabel->setAlignment(Qt::AlignCenter);
        valueLabel->setAlignment(Qt::AlignCenter);
        
        setStyleSheet(R"(
            QLabel {
                color: white;
                font: bold 12px;
            }
        )");
    }

    void setTitle(const QString &title) { titleLabel->setText(title); }
    void setRange(double min, double max) { minValue = min; maxValue = max; update(); }
    void setValue(double value) { 
        currentValue = qBound(minValue, value, maxValue); 
        valueLabel->setText(QString("%1 %2").arg(value, 0, 'f', 1).arg(unit));
        update();
    }
    void setUnit(const QString &u) { unit = u; }
    void setWarningLevels(double warn, double critical) {
        warnLevel = warn;
        criticalLevel = critical;
    }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // Calculate dimensions
        const int margin = 10;
        const int barHeight = height() - 70;
        const QRectF barRect(width()/2 - barWidth/2, margin + 20, barWidth, barHeight);
        
        // Draw background
        painter.setPen(Qt::white);
        painter.setBrush(QColor(30, 30, 30));
        painter.drawRect(barRect);
        
        // Calculate fill
        double fillHeight = barHeight * (currentValue - minValue) / (maxValue - minValue);
        QRectF fillRect = barRect.adjusted(0, barHeight - fillHeight, 0, 0);
        
        // Determine fill color
        QColor fillColor = Qt::green;
        if(currentValue >= criticalLevel) fillColor = Qt::red;
        else if(currentValue >= warnLevel) fillColor = QColor(255, 165, 0); // Orange
        
        // Draw fill
        painter.setBrush(fillColor);
        painter.drawRect(fillRect);
        
        // Draw ticks
        painter.setPen(Qt::white);
        const int tickCount = 5;
        for(int i = 0; i <= tickCount; i++) {
            double y = barRect.bottom() - (barHeight * i / tickCount);
            painter.drawLine(barRect.right() + 5, y, barRect.right() + 15, y);
            
            // Draw value
            double value = minValue + (maxValue - minValue) * (i / double(tickCount));
            painter.drawText(QRect(barRect.right() + 20, y - 10, 50, 20), 
                           Qt::AlignLeft | Qt::AlignVCenter, 
                           QString::number(value, 'f', 0));
        }
        
        // Draw current value indicator
        painter.setPen(Qt::white);
        painter.setBrush(fillColor);
        const int indicatorSize = 10;
        QPolygonF triangle;
        triangle << QPointF(barRect.left() - indicatorSize, fillRect.top() + indicatorSize/2)
                 << QPointF(barRect.left(), fillRect.top())
                 << QPointF(barRect.left() - indicatorSize, fillRect.top() - indicatorSize/2);
        painter.drawPolygon(triangle);
    }

private:
    double minValue;
    double maxValue;
    double currentValue;
    double warnLevel = 70;
    double criticalLevel = 90;
    int barWidth;
    QString unit;
    QLabel *titleLabel;
    QLabel *valueLabel;
};