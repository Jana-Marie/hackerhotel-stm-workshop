# hackerhotel-stm-workshop

## Files and notes for the ["STM32 for Arduino Users"](https://hackerhotel.nl/index.php/timetable/event/stm32-for-arduino-users/) workshop at [HackerHotel 2020](https://hackerhotel.nl/)

-----


### STM32 for Arduino Users

This workshop is intended for people who are annoyed of the chains Arduino and other electronics beginners ecosystems like MicroPhython or low.js has put on them.
We would like to show you how to use industrie's standard frameworks and toolchains to program high-performance ARM-based microcontrollers.
Most of the tools used are open source and all are free to use, so they can easily be integrated in everydays hacking projects.

This workshop is intended for people who already are familiar with low-level programming, idealy something C / C++-based like Arduino.
This workshop is not an introduction to electronics, nor programming. All tools used require Linux, so either bring your Linux laptop, or have a VM at hands that has USB hardware access.

Schedule:
- Introduction to STM32 microcontroller families
- Hardware Peripherals vs. Software BitBanging
- Introduction to CubeMX, a code generation tool for STM32 hardware init
- Setting up the compiler and flash tool
- Blinking a LED
- Introduction to DMA and advanced hardware peripherals
- Displaying stuff on a 8x8 LED matrix with zero CPU overhead
- Introduction to the GNU Project Debugger for STM32
- Experimenting and Questions

You will receive all hardware required for participating for a cost of 15€. Unfortuantly, we can only provide 10 kits consisting of an OtterPill STM32F072 based STM32 devboard, a ST-Link V2 USB debugger and  a MAX7219 LED Matrix. If you did not get a kit, you can also participate as a listener or bring your own related hardware.

If you would like to join, please prepare the following software on your laptop: 

-----

#### prerequisites (debian / Ubuntu):

```
sudo apt-get update
sudo apt-get install openocd gcc-arm-none-eabi cmake build-essential libusb-1.0-0-dev openjdk-13-jre dfu-util stlink-tools
```

(evt. sudo apt install libc6-i386 libnewlib-arm-none-eabi)

If you don't have stlink-tools, compile from scratch:

https://github.com/texane/stlink

https://github.com/texane/stlink/blob/master/doc/compiling.md


If you have arm-none-eabi-gcc V6 (check with 'arm-none-eabi-gcc - v'):
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```

##### CubeMX:

https://www.st.com/en/development-tools/stm32cubemx.html

extract, launch with

`java -jar STM32CubeMX`


##### A text editor of your choice:

e.g. Atom, Sublime, vscode


##### Workshop files:

https://github.com/Jan--Henrik/hackerhotel-stm-workshop


##### Workshop Hardware:

https://github.com/Jan--Henrik/OtterPill

-----

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

-----

#### Monochrome matrix test:

##### Settings:


*(left)*
```
System Core -> SYS -> Debug Serial Wire
```
```
Timers -> TIM1 -> Channel 1: PWM Generation CH1N or PB13 -> TIM1_CH1N
Parameter Settings -> Counter Settings -> Prescaler -> 2000
Parameter Settings -> Counter Settings -> Counter Periode -> 99
Parameter Settings -> PWM Generation Channel 1N -> Pulse -> 50
NVIC Settings -> TIM1 break, update... interrupts -> enable
```
```
Connectivity -> SPI1 -> Mode: Full-Dublex Master
Connectivity -> SPI1 -> : Hardware NSS Signal: Hardware NSS Signal Output
Parameter Settings -> Data Size -> 16
Parameter Settings -> Clock Parameter -> Prescaler -> 8
DMA Settings -> Add -> SPI1_TX 
```

*(top)*
```
Clock Configuration -> System Clock MUX, select HSI48
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


Src -> main.c (https://github.com/Jan--Henrik/hackerhotel-stm-workshop/blob/master/matrix-test-monochrome/Src/main.c)

add line 40 (/* USER CODE BEGIN PM */): 
```
void setPixel(uint8_t x, uint8_t y, uint8_t c);
```

add line 50 (/* USER CODE BEGIN PV */):
```
uint16_t fbuf[8];     
uint16_t cbuf[5]={0x0b07,   //scanLimit no limit                                     
                  0x0900,   //decode mode none                                      
                  0x0c01,   //shutdown off                                      
                  0x0f00,   //display test off                                      
                  0x0a0f};  //intensity max                    
int16_t color;                   
uint64_t lastTick;                    
uint32_t tickTime;
```

add line 109 (/* USER CODE BEGIN 2 */):
```
HAL_SPI_Init(&hspi1);
HAL_SPI_Transmit_DMA(&hspi1,cbuf,5);                                          
for(uint8_t i = 1; i <= 8; i++){                        
  fbuf[i-1] = (i << 8);                                            
  //fbuf[i-1] = fbuf[i-1] + 0xF5;                      
}                                          
HAL_TIM_Base_Start_IT(&htim1);                      
HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);                    
```

add line 129:
```
lastTick = HAL_GetTick(); 
for(uint8_t x = 0; x <= 7; x++){ 
  for(uint8_t y = 0; y <= 7; y++){ 
    color = 127.0 + sin((10.0*(((x-4)/8.0)*sin(HAL_GetTick()/500.0)+((y-4)/8.0)*cos(HAL_GetTick()/300.0)))+HAL_GetTick()/10000.0); 

    setPixel(x,y,color > 126 ? 1:0); 
    } 
  } 
tickTime = HAL_GetTick() - lastTick; 
HAL_SPI_Transmit_DMA(&hspi1,fbuf,8);
```

add line 314:
```
void setPixel(uint8_t x, uint8_t y, uint8_t c){ 
  if(x > 7 || y > 7) return; 
  c = c & 0x01; 
  fbuf[y] ^= (-c ^ fbuf[y]) & (1UL << x); 
}
```

In console type
```
make
```
Should compile just fine


##### Flashing


Wire up your led matrix: 

![](https://github.com/Jan--Henrik/hackerhotel-stm-workshop/raw/master/IMG_20200215_121521.jpg)

plug in the OtterPill while pressing the DFU button

`lsusb` should show something like:

Bus 001 Device 007: ID 0483:df11 STMicroelectronics STM Device in DFU Mode

flash with dfu-util:
```
dfu-util -a 0 -s 0x08000000:leave -D build/matrix-test-monochrome.bin
```

![](monochrome_c.gif)


-----

#### Greyscale matrix test:

##### Settings:


*(left)*
```
System Core -> SYS -> Debug Serial Wire
```
```
Timers -> TIM1 -> Channel 1: PWM Generation CH1N or PB13 -> TIM1_CH1N
Parameter Settings -> Counter Settings -> Prescaler -> 2000
Parameter Settings -> Counter Settings -> Counter Periode -> 99
Parameter Settings -> PWM Generation Channel 1N -> Pulse -> 50
NVIC Settings -> TIM1 break, update... interrupts -> enable
```
```
Connectivity -> SPI1 -> Mode: Full-Dublex Master
Connectivity -> SPI1 -> : Hardware NSS Signal: Hardware NSS Signal Output
Parameter Settings -> Data Size -> 16
Parameter Settings -> Clock Parameter -> Prescaler -> 8
DMA Settings -> Add -> SPI1_TX -> 
```

*(top)*
```
Clock Configuration -> System Clock MUX, select HSI48
```

*(top)*
```
Project Manager
select name
select location
Toolchain / IDE -> Makefile
top right: Generate Code
```

##### Toolchain


Open Terminal, locate to project location

Launch editor in project root directory

Launch console and type
```
make
```
Should compile just fine


##### Modify project. Start with:

Src -> main.c (https://github.com/Jan--Henrik/hackerhotel-stm-workshop/blob/master/matrix-test-grayscale/Src/main.c)

add line 30 (/* USER CODE BEGIN PTD */):
```
#define FBDEPH 4 
#define FBDEPHPOW 16
```

add line 49 (/* USER CODE BEGIN PV */) 
```
uint8_t modCnt = 0; 
uint16_t fbuf[FBDEPH][8]; 
uint16_t cbuf[5]={0x0b07,   //scanLimit no limit                                     
                  0x0900,   //decode mode none                                      
                  0x0c01,   //shutdown off                                      
                  0x0f00,   //display test off                                      
                  0x0a0f};  //intensity max                    
int16_t color;                   
uint64_t lastTick;                    
uint32_t tickTime;
```

add line 69 (/* USER CODE BEGIN PM */): 
```
void setPixel(uint8_t x, uint8_t y, uint8_t c);
```

add line 111 (/* USER CODE BEGIN 2 */):
```
HAL_SPI_Init(&hspi1); 
HAL_SPI_Transmit_DMA(&hspi1,cbuf,4); 

for(uint8_t i = 1; i <= 8; i++){ 
  for(uint8_t j = 0; j <= FBDEPH-1; j++){ 
    fbuf[j][i-1] = (i << 8); 
  } 
} 

HAL_TIM_Base_Start_IT(&htim1); 
HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
```

add line 131:
```
lastTick = HAL_GetTick(); 
for(uint8_t x = 0; x <= 7; x++){ 
  for(uint8_t y = 0; y <= 7; y++){ 
    color = 32.0 + sin((10.0*(((x-4)/8.0)*sin(HAL_GetTick()/500.0)+((y-4)/8.0)*cos(HAL_GetTick()/300.0)))+HAL_GetTick()/10000.0)*32; 
    //color = x*64; 
    setPixel(x,y,color); 
    } 
  } 
tickTime = HAL_GetTick() - lastTick;
```

add line 342 (/* USER CODE BEGIN 4 */):
```
void setPixel(uint8_t x, uint8_t y, int16_t c){ 
  if(x > 7 || y > 7) return; 
  if(c >= FBDEPHPOW) c = FBDEPHPOW-1; 
  if(c < 0) c = 0; 
  //c = c & 0x01; 
  uint8_t cBit; 

  for(uint8_t i = 0; i <= FBDEPH-1; i++){ 
    cBit = c & 0x01; 
    fbuf[i][y] ^= (-cBit ^ fbuf[i][y]) & (1UL << x); 
    c = c >> 1; 
  } 
}
```

Src -> stm32f0xx_it.c (https://github.com/Jan--Henrik/hackerhotel-stm-workshop/blob/master/matrix-test-grayscale/Src/stm32f0xx_it.c)

add line 35:
```
#define FBDEPH 4 
#define FBDEPHPOW 16
```

add line 64:
```
extern uint16_t fbuf[8][8]; 
extern uint8_t modCnt;
```

add in "void TIM1_BRK_UP_TRG_COM_IRQHandler(void)"
```
if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_UPDATE) && HAL_DMA_GetState(&hdma_spi1_tx) == HAL_DMA_STATE_READY){ 
  modCnt++; 
  modCnt = modCnt % FBDEPH; 
  __HAL_TIM_SET_PRESCALER(&htim1,(((modCnt+FBDEPH+1)%FBDEPH)*100)); 
  __HAL_TIM_SET_COUNTER(&htim1,0); 
  HAL_SPI_Transmit_DMA(&hspi1,&fbuf[modCnt][0],8); 
}
```

In console type
```
make
```

Should compile just fine

##### Flashing

wire up your led matrix:

![]( https://github.com/Jan--Henrik/hackerhotel-stm-workshop/raw/master/IMG_20200215_121521.jpg)

plug in the OtterPill while pressing the DFU button

`lsusb` should show something like:

Bus 001 Device 007: ID 0483:df11 STMicroelectronics STM Device in DFU Mode

flash with dfu-util:
```
dfu-util -a 0 -s 0x08000000:leave -D build/matrix-test-monochrome.bin
```

![](grayscale_c.gif)
