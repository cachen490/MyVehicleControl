#ifndef APP_H
#define APP_H

void App_Init(void);
void App_Loop(void);        /* 主循环里反复调，内部按 1ms 分频 */
void App_Task_1ms(void);    /* 1ms 周期的全部控制逻辑 */

#endif
