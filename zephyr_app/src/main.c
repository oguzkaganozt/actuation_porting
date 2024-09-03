// Simple .c file content

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static struct k_timer my_timer;

static void timer_expiry_function(struct k_timer *timer_id)
{
    static int count = 0;
    printk("Timer expired %d times\n", ++count);
}

void main(void)
{
    int ret;

    printk("Starting Zephyr example application\n");

    k_timer_init(&my_timer, timer_expiry_function, NULL);
    k_timer_start(&my_timer, K_SECONDS(1), K_SECONDS(1));

    while (1) {
        k_msleep(5000);
        printk("Main thread still running\n");
    }
}

