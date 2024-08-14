**STM32 LED Blinking Using TIM2 Timer Interrupts**
This project demonstrates how to control a sequence of LEDs on an STM32 microcontroller using hardware timer interrupts, specifically TIM2.
Unlike traditional delay-based methods, this approach uses the timer's interrupt capabilities to efficiently manage the CPU's workload.

**Overview**
The project is designed to toggle four LEDs connected to different GPIO pins of the STM32 board. Each LED lights up for one second,
then turns off as the next LED in the sequence turns on. This process is controlled by the TIM2 hardware timer, which triggers an
interrupt every second. This interrupt-driven approach allows the CPU to perform other tasks or enter low-power modes while waiting
for the next interrupt, improving overall system efficiency.

**Key Features**
    Efficient CPU Usage: By using interrupts, the CPU is not kept busy with delay loops, allowing for more efficient operation.
    Sequential LED Blinking: The LEDs are toggled in a sequence, each staying on for one second before the next one lights up.
    Flexible Configuration: The timing and behavior can be easily adjusted by reconfiguring the TIM3 settings in STM32CubeMX.

**How it works**
Every 1 second, the Timer sends an interrupt to CPU and the function called **void TIM2_IRQHandler(void)** in file stm32u5xx_it.c will be run.
In this function an external variable called led_flag will be triggered and the main function will be run.
You just need to start the Timer in interrupt mode in main.c. by using the following command:
      **HAL_TIM_Base_Start_IT(&htim2);**
--->When the timer starts, it will generate an interrupt when the counter reaches the auto-reload value.

**Clk Configuration**
To configure the timer in CubeMX : https://deepbluembedded.com/stm32-timer-interrupt-hal-example-timer-mode-lab/
