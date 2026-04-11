# softRTC

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A lightweight, software-based real-time clock (RTC) library for Arduino and compatible microcontrollers. This library allows you to maintain date and time (hours, minutes, seconds, day, month, year) **without** needing dedicated RTC hardware modules. It supports both 12-hour (AM/PM) and 24-hour modes, calculates weekdays, and offers a straightforward API for time manipulation.

---

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
  - [Basic Steps](#basic-steps)
  - [Example Sketch](#example-sketch)
- [Contributing](#contributing)
- [License](#license)
- [Author](#author)

---

## Features

- **Software RTC**: Maintains date/time using `millis()` – no additional hardware required.
- **12-Hour / 24-Hour** Modes: Easily switch between standard (AM/PM) or 24-hour (military) time.
- **Weekday Calculation**: Automatically determines the weekday (Sun–Sat).
- **Serial Output**: Built-in `print()` function for quick debugging or logging.
- **Minimal Footprint**: Designed to be lightweight for memory-constrained devices.

---

## Installation

### Manual Installation

1. Click the green “Code” button on this GitHub repository and select **Download ZIP**.
2. Extract the ZIP file on your computer.
3. Rename the extracted folder to `softRTC` (if it isn’t already).
4. Move the `softRTC` folder into your Arduino libraries directory:
    - **Windows**: `Documents\Arduino\libraries\`
    - **macOS**: `~/Documents/Arduino/libraries/`
    - **Linux**: `~/Arduino/libraries/`
5. Restart the Arduino IDE if it was open.

---

## Usage

### Basic Steps

1. **Include the Library**  
    ```cpp
    #include <softRTC.h>
    ```

2. **Create an RTC Object**  
    ```cpp
    softRTC myRTC;
    ```

3. **Set the Time**  
    ```cpp
    // write(day, month, year, hour, minute, second, isPM, is12HMode)
    // Example: set 10:15:30 AM, 21 March 2025 in 24-hour mode
    myRTC.write(21, 3, 2025, 10, 15, 30, false, MODE_24H);
    ```

4. **Read the Time**  
    ```cpp
    uint8_t d, m, h, min, s, weekday;
    uint16_t y;
    bool pm, is12;
    myRTC.read(d, m, y, h, min, s, pm, is12, weekday);
    // Now you can use these variables in your code...
    ```

5. **Print the Time**  
    ```cpp
    myRTC.print(); // e.g., "21-3-2025 10:15:30 24H Fri"
    ```

6. **Check Sync Status**  
    ```cpp
    if (!myRTC.syncStatus()) {
      // If false, the clock isn't set or has invalid data
    }
    ```

### Example Sketch

```cpp
#include <softRTC.h>

softRTC myRTC;

void setup() {
  Serial.begin(9600);
  // Set the time to 9:30:45 PM, 21 March 2025 in 12-hour mode
  myRTC.write(21, 3, 2025, 9, 30, 45, true, MODE_12H);
}

void loop() {
  // Print the current time every second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    myRTC.print();
  }
}
 ```
## Contributing

**Issues / Feature Requests**  
If you run into any problems or have ideas for improvements, please open an issue on [GitHub](https://github.com/manzarehassin/softRTC/issues).

**Pull Requests**  
- Fork the repository and create a branch for your changes.  
- Make commits with clear and descriptive messages.  
- Submit a pull request explaining what you changed and why.

**Code Style**  
Follow the existing code structure. If you add a feature, include an example or documentation update.

---

## License

This library is released under the [MIT License](LICENSE).  
See the `LICENSE` file for details.

---

## Author

**Manzar E Hassin**  
- GitHub: [manzarehassin](https://github.com/manzarehassin)  
- Email: [mnz247@hotmail.com](mailto:mnz247@hotmail.com)

Feel free to reach out with questions, feedback, or to share any projects using **softRTC**!
