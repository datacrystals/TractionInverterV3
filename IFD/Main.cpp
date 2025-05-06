#include <QApplication>
#include <QMainWindow>
#include <QtWidgets>
#include <QTimer>

class TrainHMI : public QMainWindow {
    Q_OBJECT

public:
    TrainHMI(QWidget *parent = nullptr) : QMainWindow(parent) {
        setupUI();
        setupSimulation();
    }

private:
    QLCDNumber *speedDisplay;
    QLCDNumber *tempDisplay;
    QLabel *headlightIndicator;
    QLabel *doors[4];
    QLabel *emergencyBrakeIndicator;

    void setupUI() {
        // Set main window properties
        setWindowTitle("Train HMI");
        setStyleSheet("background-color: black;");

        // Create central widget and main layout
        QWidget *centralWidget = new QWidget(this);
        QGridLayout *mainLayout = new QGridLayout(centralWidget);
        
        // Create and add all display groups
        mainLayout->addWidget(createSpeedGroup(), 0, 0);
        mainLayout->addWidget(createTemperatureGroup(), 0, 1);
        mainLayout->addWidget(createHeadlightGroup(), 1, 0);
        mainLayout->addWidget(createDoorStatusGroup(), 1, 1);
        mainLayout->addWidget(createEmergencyBrakeIndicator(), 2, 0, 1, 2);

        setCentralWidget(centralWidget);
    }

    QGroupBox* createSpeedGroup() {
        QGroupBox *group = new QGroupBox("Speed (km/h)");
        QVBoxLayout *layout = new QVBoxLayout;
        
        speedDisplay = new QLCDNumber;
        speedDisplay->setDigitCount(3);
        speedDisplay->setSegmentStyle(QLCDNumber::Filled);
        speedDisplay->setStyleSheet("color: #00FF00; background-color: #001100;");
        
        layout->addWidget(speedDisplay);
        group->setLayout(layout);
        group->setStyleSheet("QGroupBox { color: white; font: bold 14px; }");
        return group;
    }

    QGroupBox* createTemperatureGroup() {
        QGroupBox *group = new QGroupBox("Cabin Temperature (°C)");
        QVBoxLayout *layout = new QVBoxLayout;
        
        tempDisplay = new QLCDNumber;
        tempDisplay->setDigitCount(2);
        tempDisplay->setSegmentStyle(QLCDNumber::Filled);
        tempDisplay->setStyleSheet("color: #00FFFF; background-color: #001111;");
        
        layout->addWidget(tempDisplay);
        group->setLayout(layout);
        group->setStyleSheet("QGroupBox { color: white; font: bold 14px; }");
        return group;
    }

    QGroupBox* createHeadlightGroup() {
        QGroupBox *group = new QGroupBox("Headlights");
        QVBoxLayout *layout = new QVBoxLayout;
        
        headlightIndicator = new QLabel;
        headlightIndicator->setFixedSize(50, 50);
        headlightIndicator->setStyleSheet("background-color: #002200; border-radius: 25px;");
        
        layout->addWidget(headlightIndicator, 0, Qt::AlignCenter);
        group->setLayout(layout);
        group->setStyleSheet("QGroupBox { color: white; font: bold 14px; }");
        return group;
    }

    QGroupBox* createDoorStatusGroup() {
        QGroupBox *group = new QGroupBox("Doors Status");
        QGridLayout *layout = new QGridLayout;
        
        QStringList doorLabels = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
        for(int i = 0; i < 4; i++) {
            doors[i] = new QLabel;
            doors[i]->setFixedSize(30, 30);
            doors[i]->setStyleSheet("background-color: red; border-radius: 15px;");
            layout->addWidget(new QLabel(doorLabels[i]), i/2, (i%2)*2);
            layout->addWidget(doors[i], i/2, (i%2)*2+1);
        }
        
        QLabel *doorStatusLabel = new QLabel("CLOSED");
        doorStatusLabel->setStyleSheet("color: white; font: bold 16px;");
        layout->addWidget(doorStatusLabel, 2, 0, 1, 4, Qt::AlignCenter);
        
        group->setLayout(layout);
        group->setStyleSheet("QGroupBox { color: white; font: bold 14px; }");
        return group;
    }

    QLabel* createEmergencyBrakeIndicator() {
        emergencyBrakeIndicator = new QLabel("EMERGENCY BRAKE");
        emergencyBrakeIndicator->setAlignment(Qt::AlignCenter);
        emergencyBrakeIndicator->setStyleSheet(
            "color: red; font: bold 24px; background-color: #220000;"
            "padding: 20px; border: 2px solid red;"
        );
        return emergencyBrakeIndicator;
    }

    void setupSimulation() {
        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, [this]() {
            // Simulate changing values
            static bool toggle = false;
            speedDisplay->display(rand() % 120);
            tempDisplay->display(20 + rand() % 10);
            headlightIndicator->setStyleSheet(toggle ? 
                "background-color: #00FF00; border-radius: 25px;" :
                "background-color: #002200; border-radius: 25px;");
            
            for(auto &door : doors) {
                door->setStyleSheet(toggle ? 
                    "background-color: #00FF00; border-radius: 15px;" :
                    "background-color: red; border-radius: 15px;");
            }
            
            toggle = !toggle;
        });
        timer->start(1000);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    TrainHMI window;
    window.setWindowState(Qt::WindowFullScreen);
    window.show();

    return app.exec();
}

#include "Main.moc"