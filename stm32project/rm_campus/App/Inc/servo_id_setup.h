#ifndef SERVO_ID_SETUP_H
#define SERVO_ID_SETUP_H

/* 改号版 USB 任务入口，USB CDC 初始化完成后调用；此任务独占 UART7。 */
void ServoIdSetup_Task(void);

#endif
