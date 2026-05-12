
http://images.google.de/imgres?imgurl=https%3A%2F%2Fi.ytimg.com%2Fvi%2Fj4OmVLc_oDw%2Fmaxresdefault.jpg&imgrefurl=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3Dj4OmVLc_oDw&h=720&w=1280&tbnid=5YpcLhJMt6wdRM%3A&docid=rap7-7k-tsXYqM&ei=RQSvV9HFKeKU6ASa1YOgDA&tbm=isch&client=firefox-b&iact=rc&uact=3&dur=5623&page=5&start=111&ndsp=29&ved=0ahUKEwiR_MDXpL7OAhViCpoKHZrqAMQ4ZBAzCEEoHjAe&bih=925&biw=1186

Ball on Plate Control
http://images.google.de/imgres?imgurl=http%3A%2F%2Fwww.hobbyprojects.com%2Fprojects%2Fimages%2Fball-on-plate-diagram.jpg&imgrefurl=http%3A%2F%2Fwww.hobbyprojects.com%2Fprojects%2Fball-on-plate-control.html&h=286&w=606&tbnid=w2nalg0MMkgnuM%3A&docid=eiwe_9Mw_7RJVM&ei=vQavV-TqMcWwsQHV8a3gAg&tbm=isch&client=firefox-b&iact=rc&uact=3&dur=5828&page=1&start=0&ndsp=25&ved=0ahUKEwikvfeEp77OAhVFWCwKHdV4CywQMwgmKAUwBQ&bih=925&biw=1186


161003

In SendData() camera scaling factor as NUMERIC CONSTANT !!!
  sendFloat('m', X->ssvEstPred[0]*1380);      //11 Byte
  sendFloat('n', X->ssvEstPred[1]*1380);      //11 Byte
  sendFloat('o', Y->ssvEstPred[0]*1380);      //11 Byte
  sendFloat('p', Y->ssvEstPred[1]*1380);      //11 Byte
                                              // ---------
                                              // 130 Byte pro Abtastzyklus




Here all relevant information to understand, build and use the current
project should be given, plus author & date:


0. Project Name / Author:  BalanceBoard WS2014 by Ingo Lienhardt

1. Purpose and Scope of Application 'BalanceBoard'

   Portable STM32F10x Sample Program, e.g. for Real-Time Systems Lab 

   Base program for controlling a ...

   Input:   Console/keyboard input (RS-232, 'COM')
   Output:  Console/terminal output


2. Target Hardware Variants & Relevant Target Configuration Options

   Keil MCBSTM32  Evalboard

 

3. General Compiler/Preprocessor Symbols & Include Path Settings

   For STM32F103RB on MCBSTM32:
   USE_STDPERIPH_DRIVER, STM32F10X_MD, USE_MCBSTM32, __CC, _ARM  

   For STM32F107VC on MCBSTM32C:
   USE_STDPERIPH_DRIVER, STM32F10X_CL, USE_MCBSTM32C, __CC, _ARM  

   Include paths settings for STM32F103RB && STM32F107VC:
   .\; .\init; .\cfg; ..\StdPeriph_Lib\STM32F10x_StdPeriph_Driver\inc;
                     ..\StdPeriph_Lib\CMSIS\CM3\DeviceSupport\ST\STM32F10x


4. Startup Files Used 

 [ startup_stm32f10x_md.asm   for STM32F103RB on MCBSTM32 ]
   startup_stm32f10x_cl.asm   for STM32F107VC on MCBSTM32C


5. Peripherals Used & Relevant Configuration Options

   RCC

   USART#2 : Serial IO with image processing PC ('COM1') at 115,200Bd 8N1,
             TxD = PD#5,  RxD = PD#6

   GPIO_B  : PWM signals for servo actuators (Bit#8 - Bit#9 alternate function outputs)

   TIMER#4 : PWM signals generation 
             TIM4.PSC = 72          i.e. 1 MHz PSC-clock, with 1us resolution
             TIM4.ARR = 9999        i.e. 10ms repeat period
             Channel3 in PWM1 Mode for x-axis actuator, 5ms pulse width after init
             Channel4 in PWM1 Mode for x-axis actuator, 2.5ms pulse width after init


 [ GPIO_E  or GPIO_D for LED-IO (Bit#8 - Bit#15 as push-pull outputs)  ]
 [ USART#2 or USART6 for console-IO ('COM'; at 115200, 8,N,1)          ]


2016/July/11 Birger Mysliwetz 
