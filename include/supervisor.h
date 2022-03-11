#pragma once

#define SUPERVISOR_WATCHDOG_TIMEOUT_SEC 60
#define SUPERVISOR_TASK_LOOP_DELAY_MS ((SUPERVISOR_WATCHDOG_TIMEOUT_SEC - 3) * 1000)
// SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB is the min. number of free KB any task may have in its stack before the supervisor reboots.
#define SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB 2
// SUPERVISOR_STUCK_PROGRESS_THRESHOLD is the max number of consecutive readings the supervisor may observe from a critical parameter before it reboots.
// It must be sufficiently high in order to accomodate the sleeping periods of individual tasks.
#define SUPERVISOR_STUCK_PROGRESS_THRESHOLD ((2 * LORAWAN_TX_KINDS * POWER_SLOWEST_TX_INTERVAL_SEC * 1000) / SUPERVISOR_TASK_LOOP_DELAY_MS + 2)

void supervisor_setup();
void supervisor_reset();
void supervisor_check_task_stack();
void supervisor_check_gps();
void supervisor_check_lorawan();
void supervisor_check_wifi();
void supervisor_check_env_sensor();
void supervisor_check_power();
void supervisor_task_loop(void *_);