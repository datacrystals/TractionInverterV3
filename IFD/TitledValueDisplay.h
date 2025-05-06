#pragma once
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

class TitledValueDisplay : public QWidget {
    Q_OBJECT
public:
    explicit TitledValueDisplay(QWidget *parent = nullptr)
        : QWidget(parent), titleLabel(new QLabel), valueLabel(new QLabel), unitLabel(new QLabel) {
        QVBoxLayout *layout = new QVBoxLayout(this);
        
        titleLabel->setAlignment(Qt::AlignCenter);
        valueLabel->setAlignment(Qt::AlignCenter);
        unitLabel->setAlignment(Qt::AlignCenter);

        layout->addWidget(titleLabel);
        layout->addWidget(valueLabel);
        layout->addWidget(unitLabel);

        setStyleSheet(R"(
            QLabel {
                color: white;
                font: bold 12px;
            }
        )");
    }

    void setTitle(const QString &text) { titleLabel->setText(text); }
    void setValue(const QString &text) { valueLabel->setText(text); }
    void setUnit(const QString &text) { unitLabel->setText(text); }
    void setValueColor(const QColor &color) {
        valueLabel->setStyleSheet(QString("color: %1; font: bold 18px;").arg(color.name()));
    }

private:
    QLabel *titleLabel;
    QLabel *valueLabel;
    QLabel *unitLabel;
};