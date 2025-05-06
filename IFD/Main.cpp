#include <QApplication>
#include <QMainWindow>
#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <QProgressBar>
#include "TitledValueDisplay.h"
#include "VerticalBarGauge.h"

class TrainHMI : public QMainWindow {
    Q_OBJECT
public:
    TrainHMI(QWidget *parent = nullptr) : QMainWindow(parent) {
        setupUI();
        setupSimulation();
        setWindowState(Qt::WindowFullScreen);
    }

private:
    void setupUI() {
        QWidget *centralWidget = new QWidget;
        QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
        mainLayout->setSpacing(2);

        // Style setup
        QString groupBoxStyle = R"(
            QGroupBox {
                border: 2px solid white;
                border-radius: 5px;
                margin-top: 10px;
                color: white;
                font: bold 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
        )";

        // Pressure Section
       // In setupUI() replace pressure group with:
        QGroupBox *pressureGroup = new QGroupBox("Air Brake Pressures (psi)");
        pressureGroup->setStyleSheet(groupBoxStyle);
        QGridLayout *pressureLayout = new QGridLayout;
        QStringList pressureLabels = {"Main Compressor", "Equalizing Res", "Master Cylinder", 
                                    "Brake Pipe", "EOT Pressure"};
        for(int i = 0; i < 5; i++) {
            pressureGauges[i] = new VerticalBarGauge;
            pressureGauges[i]->setTitle(pressureLabels[i]);
            pressureGauges[i]->setRange(0, 200);
            pressureGauges[i]->setUnit("psi");
            pressureGauges[i]->setWarningLevels(130, 150);
            pressureLayout->addWidget(pressureGauges[i], 0, i);
        }
        pressureGroup->setLayout(pressureLayout);


        // Voltage/Power Section
        QGroupBox *voltageGroup = new QGroupBox("Electrical System");
        voltageGroup->setStyleSheet(groupBoxStyle);
        QGridLayout *voltageLayout = new QGridLayout;
        QStringList voltageLabels = {"Battery V", "DC Link V", "Battery A", 
                                   "Inverter A", "DynBrake A", "DynBrake kW"};
        for(int i = 0; i < 6; i++) {
            voltageDisplays[i] = new TitledValueDisplay;
            voltageDisplays[i]->setTitle(voltageLabels[i]);
            voltageLayout->addWidget(voltageDisplays[i], 0, i);
        }
        voltageGroup->setLayout(voltageLayout);

        // Temperature Section
        QGroupBox *tempGroup = new QGroupBox("Temperatures (°C)");
        tempGroup->setStyleSheet(groupBoxStyle);
        QGridLayout *tempLayout = new QGridLayout;
        QStringList tempLabels = {"Traction Motor", "Boost Converter", "Inverter", 
                                "Batteries", "Ambient"};
        for(int i = 0; i < 5; i++) {
            tempDisplays[i] = new TitledValueDisplay;
            tempDisplays[i]->setTitle(tempLabels[i]);
            tempLayout->addWidget(tempDisplays[i], 0, i);
        }
        tempGroup->setLayout(tempLayout);

        // Add sections to main layout
        mainLayout->addWidget(pressureGroup);
        mainLayout->addWidget(voltageGroup);
        mainLayout->addWidget(tempGroup);

        // Styling
        centralWidget->setStyleSheet("background-color: black;");
        setCentralWidget(centralWidget);
    }

    void setupSimulation() {
        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, [this]() {
            // Simulate pressure values
            // In setupSimulation():
            static std::array<double, 5> pressures = {150, 110, 90, 85, 80};
            for(int i = 0; i < 5; i++) {
                pressures[i] += (rand() % 5 - 2);
                pressureGauges[i]->setValue(pressures[i]);
            }

            // Simulate electrical values
            static std::array<double, 6> voltages = {600, 610, 150, 145, 75, 25};
            for(int i = 0; i < 6; i++) {
                voltages[i] += (rand() % 10 - 5);
                voltageDisplays[i]->setValue(QString::number(voltages[i], 'f', 1));
                voltageDisplays[i]->setUnit(i < 2 ? "V" : (i < 5 ? "A" : "kW"));
                voltageDisplays[i]->setValueColor(getVoltageColor(i, voltages[i]));
            }

            // Simulate temperatures
            static std::array<double, 5> temps = {65, 45, 55, 35, 25};
            for(int i = 0; i < 5; i++) {
                temps[i] += (rand() % 3 - 1);
                tempDisplays[i]->setValue(QString::number(temps[i], 'f', 1));
                tempDisplays[i]->setValueColor(getTempColor(temps[i]));
            }
        });
        timer->start(500);
    }

    QColor getTempColor(double temp) {
        if(temp > 80) return Qt::red;
        if(temp > 60) return QColor(255, 165, 0); // Orange
        if(temp > 40) return Qt::yellow;
        return Qt::green;
    }

    QColor getPressureColor(double psi) {
        if(psi > 150) return Qt::red;
        if(psi > 130) return Qt::yellow;
        return Qt::green;
    }

    QColor getVoltageColor(int index, double value) {
        if(index < 2) { // Voltages
            if(value < 580 || value > 620) return Qt::red;
            if(value < 590 || value > 610) return Qt::yellow;
            return Qt::green;
        }
        return Qt::green;
    }

    TitledValueDisplay *pressureDisplays[5];
    TitledValueDisplay *voltageDisplays[6];
    TitledValueDisplay *tempDisplays[5];
    VerticalBarGauge *pressureGauges[5];
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    TrainHMI window;
    window.show();
    return app.exec();
}

#include "Main.moc"