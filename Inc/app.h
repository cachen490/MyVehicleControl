#ifndef APP_H
#define APP_H

void App_Init(void);        /* 上电调一次 */
void App_Loop(void);        /* 主循环里无限调用 */
void App_Task_1ms(void);    /* 1ms 周期的全部控制逻辑 */

#endif /* APP_H */
