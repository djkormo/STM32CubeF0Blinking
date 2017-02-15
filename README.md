# STM32CubeF0Blinking
First try of using DAC i ADC  for FM synthesis Cortex M0, Hal Library

First try to use DAC with TIM6 as source trigger to produre sum of three sine waves for audio applications.

Sine wave wille be generating using DDS algorythm
http://lancasterhunt.co.uk/direct-digital-synthesis-dds-for-idiots-like-me/

Frequencies are controlled by one resistor per channel connected to ADC (channel 0.1 and 2) .
Pots are connected to PA0, PA1, PA2 pins.




It was made with

Eclipse for C++ developers https://eclipse.org/downloads/packages/eclipse-ide-cc-developers/keplersr2

Eclipse CDT plugin http://www.eclipse.org/cdt/

Sourcery CodeBench Lite Edition

https://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/


Initial template for eclipse with HAL library was made from STM32CubeMx  according to 

http://www.carminenoviello.com/2015/11/02/quickly-import-stm32cubemx-project-eclipse-project/


The program was tested on

STM32F0 Discovery board

http://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-discovery-kits/stm32f0discovery.html

with STM Studio
