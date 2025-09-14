# Dreidel M5 - ESP32 Dreidel Game

An 8-bit style dreidel simulator for the M5StickC-PLUS ESP32 device with music, graphics, and authentic gameplay.

## Features

- **Interactive Dreidel Spinning**: Press Button A to spin the dreidel
- **Authentic Hebrew Symbols**: Nun (נ), Gimel (ג), Hey (ה), and Shin (ש)
- **Traditional Music**: Plays the classic dreidel song during spinning
- **Colorful Graphics**: Custom BMP images for each dreidel face and game state
- **Hardware Integration**: Uses M5StickC-PLUS LCD, buzzer, and button

## Hardware Requirements

- **M5StickC-PLUS** ESP32 development board
- USB-C cable for programming and power

## Game Rules

The dreidel has four Hebrew letters, each with a traditional meaning in the dreidel game:

- **Nun (נ)** - "Nothing" - No action
- **Gimel (ג)** - "Get All" - Take everything from the pot
- **Hey (ה)** - "Half" - Take half the pot
- **Shin (ש)** - "Put In" - Add to the pot

## Hardware Specifications

### M5StickC-PLUS Pinout Used
- **Button A**: GPIO 37 (spin trigger)
- **Buzzer**: GPIO 2 (music output)
- **LCD**: ST7789 135x240 display via SPI
  - MOSI: GPIO 15
  - CLK: GPIO 13
  - CS: GPIO 5
  - DC: GPIO 23
  - RST: GPIO 18
- **Power Management**: AXP192 PMIC via I2C (GPIO 21/22)

## Building and Flashing

### Prerequisites
- ESP-IDF v4.4 or later
- USB driver for M5StickC-PLUS

### Build Instructions

#### Using Makefile (Recommended)
```bash
# Clean any previous builds
make clean

# Build the project
make build

# Flash to device (default port COM4)
make flash

# Flash and monitor
make flash-monitor

# Override default port
make flash PORT=COM3

# See all available targets
make help
```

#### Using ESP-IDF directly
```bash
# Clean any previous builds
idf.py fullclean

# Build the project
idf.py build

# Flash to device (replace COM4 with your port)
idf.py -p COM4 -b 115200 flash monitor
```

### Find Your Device Port
- **Windows**: Check Device Manager for COM port
- **macOS**: `ls /dev/cu.usbserial-*`
- **Linux**: `ls /dev/ttyUSB*`

## Project Structure

```
dreidel_m5/
├── main/
│   ├── dreidel_m5.c          # Main game logic
│   └── CMakeLists.txt        # Component build config
├── images/
│   └── dreidel/              # BMP graphics assets
│       ├── intro.bmp         # Welcome screen
│       ├── prompt.bmp        # "Press A" prompt
│       ├── base.bmp          # Dreidel base
│       ├── left.bmp          # Spinning animation frame
│       ├── right.bmp         # Spinning animation frame
│       ├── nun.bmp           # Nun result screen
│       ├── gimel.bmp         # Gimel result screen
│       ├── hey.bmp           # Hey result screen
│       └── shin.bmp          # Shin result screen
├── CMakeLists.txt            # Main build config
├── sdkconfig                 # ESP-IDF configuration
└── README.md                 # This file
```

## Game Flow

1. **Welcome Screen**: Shows intro graphics and "Press A" prompt
2. **Spin Animation**: Fast-switching graphics with dreidel music
3. **Result Display**: Shows the landed symbol with name and meaning
4. **Return to Waiting**: Ready for the next spin

## Technical Details

- **Random Generation**: Uses ESP32 hardware RNG for fair results
- **Graphics**: 16-bit RGB565 BMP images embedded in firmware
- **Audio**: PWM-generated tones on built-in buzzer
- **Display**: ST7789 LCD with proper M5StickC-PLUS offsets
- **Power**: AXP192 PMIC initialization for stable operation

## Development Notes

- Graphics are embedded as binary data at compile time
- Music and animation run concurrently using FreeRTOS tasks
- Button debouncing prevents accidental double-spins
- Debug logging available via serial monitor

## Troubleshooting

### Build Issues
- Run `idf.py fullclean` if getting project name mismatches
- Ensure ESP-IDF environment is properly sourced

### Flashing Issues
- Check USB cable and port permissions
- Try different baud rates: 460800, 230400, 115200
- Hold Boot button during flashing if needed

### Hardware Issues
- If display is blank, check AXP192 PMIC initialization
- If no sound, verify GPIO 2 buzzer connection
- Button requires external pull-up (built into M5StickC-PLUS)

## License

This project is open source. Feel free to modify and distribute.

## Acknowledgments

- Traditional dreidel game rules and music
- M5Stack community for hardware documentation
- ESP-IDF framework by Espressif Systems