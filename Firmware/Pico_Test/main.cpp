#include "ucc5870_interface.h"

int main_old() {
    ucc5870_init();
    enter_configuration();
    set_desat_threshold(1000);
    exit_configuration();
    while (true) {
        start();
        read_status();
        sleep_ms(5000);
        stop();
        sleep_ms(5000);
    }
}