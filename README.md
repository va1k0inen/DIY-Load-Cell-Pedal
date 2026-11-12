# DIY-Load-Cell-Pedal

For clarity, I'm using a STM32F411CEU6 (WeAct Studio)

"s" stands for STABLE

"t" stands for TEST (exactly means not stable, made only for experiment)

p.s: das wurde völlig vom Claude gemacht xD

| **Version**  | **Description | **What was fixed?** |  
|--|--|--|
| v1.0s |  **Code was added with basic logic for reading data from the load cell and displaying raw values on the SSD1306 screen.** | - |
| v1.1t | **Added USB HID support and ADC input for joystick emulation.**| CUSTOM_HID_FS_BINTERVAL from 0x5 to 0x1; added another value smoothing† |

**v1.1t**
† - value from HID is floating for a little bit, here is an example from telemetry (AC1):
<img width="1201" height="941" alt="1794490217813_image" src="https://github.com/user-attachments/assets/35dc6b1b-6829-404a-b02d-62907637fcfa" />

