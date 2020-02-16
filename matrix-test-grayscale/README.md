###  workshop notes tl;dr


CubeMX, file -> New Project

Select STM32F072C8Tx, LQFP48

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

#### Expected output:

![](grayscale_c.gif)
