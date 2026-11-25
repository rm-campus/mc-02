#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include <stdbool.h>
/* StatusTask 独占蜂鸣器；上电保持静音。 */
void Buzzer_Set(bool enabled);
#endif
