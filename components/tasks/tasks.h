#pragma once

/**
 * tasks_start - create and pin all FreeRTOS application tasks.
 * Call once from app_main after bsp_init() and peripheral inits.
 */
void tasks_start(void);
