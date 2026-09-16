#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(grasplink_controller, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("GraspLink embedded controller booted");
    LOG_INF("Next: bind board-specific GPIO, ADC, I2C and UDP transport");

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
