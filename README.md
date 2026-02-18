# Guition Knob V1: ESP32-S3 Audio Line-Out "Bridge" Hack

This repository contains the hardware modification details and firmware to enable audio output from the **ESP32-S3** to the onboard **Line-Out jack** on the Guition V1 Knob.

## The Problem
The factory design of the Guition Knob V1 uses a dual-MCU setup that is... let's call it "unique":
* **ESP32-S3:** Controls the UI, Display, and Haptic motor.
* **Secondary ESP32:** Connected to the DAC (PCM5100A).
* **The Issue:** The S3 (where your main application likely lives) has no direct physical connection to the DAC. In the stock configuration, the S3 handles the UI while the ESP32 handles BT/WiFi audio independently.

## The Solution: The "I2S Repeater" Hack
By repurposing existing connections and adding a single hardware bridge, we can create a high-speed digital audio path from the S3 to the Line-Out jack.

1.  **Repurpose UART:** The existing UART communication lines between the two MCUs are reused as I2S **BCK** and **LRCK** lines.
2.  **Hardware Bridge (The Mod):** By adding a single `0402` 0R resistor (or a simple solder bridge), we link **GPIO41 (S3)** to **GPIO2 (ESP32)**. This serves as the **I2S Data** line.
3.  **Software Bridge:** The secondary ESP32 is flashed with a small "Repeater" sketch. It listens for I2S data on its input pins and immediately pipes it out to the DAC.

## Hardware Modification Detail

Locate the empty resistor pads near the secondary ESP32. You need to bridge the pad to its neighbor to connect the S3's GPIO41 signal to the ESP32's GPIO2.

| Macro View | Micro Detail (The Bridge) |
| :---: | :---: |
| ![Board Overview](./image_d80040.jpg) | ![Solder Bridge](./image_d8009b.jpg) |

## Pin Mapping
| Signal | ESP32-S3 (Source) | ESP32 (Repeater) |
| :--- | :--- | :--- |
| **I2S BCK** | Repurposed UART | Repurposed UART |
| **I2S LRCK** | Repurposed UART | Repurposed UART |
| **I2S DATA** | **GPIO41** | **GPIO2** (via Mod) |

## Software Setup

### 1. Secondary ESP32 (The "Bridge")
Flash the code found in `/repeater-firmware`. This code initializes two I2S ports:
* **Input:** Receives data from the S3.
* **Output:** Sends data to the PCM5100A.

### 2. Main ESP32-S3 (Your App)
In your code (using ESP-ADF, AudioTools, or the standard I2S driver), set your I2S output pins to the repurposed UART pins and GPIO41 for Data.

---
*Found this useful? Feel free to star the repo!*
