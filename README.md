本文件是关于stm32f407vet6的FreeRTOS工程



工程内只做了FreeRTOS的基本配置,除了使用TIM4作为HAL库的是时钟(FreeRTOS要求另外给HAL分配时钟)以外未做关于外设的任何内容



为了之后开发方便,对于FreeROTS本身和工程格式方面做了如下两点改动:

1. 将分配给FreeRTOS的堆内存增加为了32768
2. 生成的文件.c和.h文件分开(因此,任务应在freertos.c中添加)



