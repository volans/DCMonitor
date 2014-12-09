DCMonitor
=========

DCMonitor is a total solution to evaluate the power consumption of electronic devices, especially for the devices with Li-Battery. Actually, you can do more with this solution. For example, you can make it work as an accurate power supplier. I only provide a base of this project, you can improve it or change it to any direction as wide as you can. The base of this solution provide below features:

  1.0~5V@3A power supplier;
  <10uA current measurement accuracy;
  USB2.0 port for PC control;
  Bluetooth 4.0 with BLE support for mobile phone control;
  ...to be added.
  
Why DCMonitor?

There are many power consumption measurement solutions in the world, such as Angilent N6705 for the high end users, and the “PowerMonitor” from Monsoon Solutions for flexible users. But these kind of solutions cannot fit my using scenario sometimes. N6705 is too heavy and complex. I have two PowerMonitor in hand, they are small enough and provide powerful APIs for customization too, but I have to take a laptop when I am doing a filed test. BTW, I would love to control more... so I decide to make my own one and name it as DCMonitor.

Before starting up, I assembled one of my PowerMonitor in hand. I had to say Monsoon Solutions did a real good job on this small equipment although I did not understand the details. The PCB was very complex with many many logical devices and a PIC microprocessor. To be honest, it beyond my capability to get any small tips about their design. It was really good, because I can make a real personal one without impacts from existed product. Whatever, I copied part of the idea.

Draft Hardware Design

From global view, the whole design was constructed by three part, controller, power supplier and load current measurer.

Controller, I chose a STM32F103 low end processor. It provide USB2.0 full speed internally. And I use a Bluetooth4.0 module to connect to the USART of STM32 to provide BLE interface to mobile phone. The IAR is the best IDE to develop STM32 and it provide 32K memory limited evaluate licenses, which is good enough for this design on STM32F103.

Power Supplier, there are two power supplier, one fix output one to provide power to controller and other on board components. Another one should be adjustable to provide 1.0V~5V for external load. To reach a good accuracy of current measurement, a liner power chip should be selected other than switch type chip, otherwise the noise will beyond the low measuring range of ADC and meaningless. I chose MIC29151-3.3V to provide 3.3V power on board, and chose MIC29502WT to provide adjustable power supply for external. BTW, the adjustable logic is pretty complex, a 12bit DAC and a differential amplifier were took into my design.

Load current measurer, I used a AD7714(24bit ADC from ADI).

Software architecture
FreeRTOS and STM32 formal firmware will be used to build up the firmware of DCMonitor. From PC side, python+pyusb+qt+matplotlib will be used to provide multi-platform support, which means DCMonitor can be controlled in Windows, Linux or Mac. The whole transform protocols will be opened, you can make any client as you want. From mobile phone part, iOS or Android, it will be a standard UART port through BLE, and this project provide a demo on Android.

All opened materials of this project

  DCMonitor Schematic;
  DCMOnitor PCB Placement;
  DCMonitor Firmware(IAR project);
  DCMonitor PC Client(Eric project); "Eric is a pyQT IDE"
  DCMonitor Mobile Client(Android APK full source);
