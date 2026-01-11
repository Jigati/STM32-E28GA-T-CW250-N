# STM32 LCD Display Driver

Driver for E28GA-T-CW250-N 240x320 TFT LCD using 8-bit parallel interface.

## Hardware
- **MCU**: STM32F410RB (Nucleo Board)
- **Display**: E28GA-T-CW250-N (240x320, RGB565)
- **Interface**: 8-bit parallel

## Pin Connections
| LCD Pin | STM32 Pin | Function |
|---------|-----------|----------|
| DB0-DB7 | PC0-PC7   | Data bus |
| CS      | PA4       | Chip Select |
| DC      | PA5       | Data/Command |
| WR      | PA6       | Write |
| RD      | PA7       | Read |
| RESET   | PB0       | Reset |
| LED     | PB10      | Status LED |

## Features
✅ RGB565 color support  
✅ Text rendering (5x7 font)  
✅ Basic graphics (pixels, rectangles)  
✅ Demo animations  
✅ Status LED indicator  

## Build Instructions
1. Open in STM32CubeIDE
2. Build project (Ctrl+B)
3. Flash to Nucleo board
4. Enjoy!

## Demo Modes
- Color bars display
- Text rendering
- Moving box animation

## License
Free to use and modify

## Author
Jigati - January 2026