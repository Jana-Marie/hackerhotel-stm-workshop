
###  workshop notes tl;dr


CubeMX, file -> New Project

Select STM32F072C8Tx, LQFP48

-----

#### Simple blink:

##### Settings:

*(left)*
```
System Core -> SYS -> Debug Serial Wire
PB13 -> GPIO Output
PB13 -> right-click -> Enter user label -> "LED"
```

*(top)*
```
Project Manager
select name
select location
Toolchain / IDE -> Makefile
```

*(top right)*
```
Generate Code
```

##### Toolchain

Open Terminal, locate to project location

Launch editor in project root directory
```
make
```
Should compile just fine


##### Modify project. Start with:


Src -> main.c (https://github.com/Jan--Henrik/hackerhotel-stm-workshop/blob/master/blink/Src/main.c)


add line 100:
```
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1); 
HAL_Delay(100); 
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); 
HAL_Delay(100);
```

In console type 
```
make
```
Should compile just fine


##### Flashing


plug in the OtterPill while pressing the DFU button

`lsusb` should show something like:

Bus 001 Device 007: ID 0483:df11 STMicroelectronics STM Device in DFU Mode

flash with dfu-util:
```
dfu-util -a 0 -s 0x08000000:leave -D build/blink.bin
```
