# RTOS-Based-Multi-Sensor-Environmental-Analyzer

# Project Description

This project monitors ambient conditions using acquired temperature, humidity, pressure and gas sensor data. When the acquired data exceeds predefined thresholds and the ambient is assessed as unhealthy, the system triggers a loud buzzer to alert the user. Additionally, it continuously provides real-time information about the ambient conditions via the dashboard.

# Software Architecture

The system operates based on event-driven task structure using CMSIS-RTOS v2. Below is a breakdown of the  architecture:

* __STM32 Side__

A sound sensor was connected to an external interrupt pin. When a sound is detected, an interrupt is triggered and a binary semaphore is released. A LED is turned on to indicate that the interrupt has occured.

The sensor task waits until the binary semaphore is acquired. Once acquired, it initiates the measurement process.

__The BME280 sensor__ is used to measure temperature, humidity and pressure. A driver sourced from GitHub was adapted for RTOS compatibility (e.g., with mutex protection during I2C access). No additional filtering was applied because the sensor already includes internal filtering from the factory.

__The MQ135 sensor__ is used to measure gas quality. A Moving Average Filter was implemented to obtain more reliable and stable data. The sensor's raw output is inversely proportional to air quality: a higher analog value indicates poor air quality. To present results as a percentage (where 100% represents clean air), the filtered value is subtracted from 100.

Sensor readings are taken every 3 seconds using `vTaskDelayUntil`. Readings sensor data is sent to a message queue. 

An UART task reads the data from the queue. Values are converted to a JSON format using `snprintf`. UART communication is protected using a mutex and the data is sent via UART.

* __ESP32 Side__

The ESP32 receives the JSON-formatted data from the STM32 via UART, parses it, and publishes it to the Adafruit IO Dashboard using the MQTT protocol. On the ESP32 side, the received data is also used for ambient evaluation. If parameters fall outside the predefined thresholds (e.g., temperature < 15°C and humidity < 50%), a warning is triggered. A buzzer is activated when the ambient conditions are poor. These evaluation messages are also published to the dashboard for user feedback.

__Hardware Note__: The BME280 sensor module used in this project was hand-soldered. Although the soldering wasn’t perfectly clean visually, continuity was tested, and no short circuits were found. The sensor has been functioning correctly under real-time operating conditions.

Figure 1 : System Overview

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/system_overview.jpeg" alt="System Overview" width="500"/>

Figure 2 : Adafruit IO Dashboard

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/dashboard.png" alt="Dashboard" width="500"/>

# Hardware Components

* __STM32F0DISC__ – Main microcontroller unit for sensor acquisition and processing

* __ESP32 WROOM 32D__ – Wi-Fi module for MQTT communication with Adafruit IO  

* __USB to TTL (Serial) Communication Module__ – For debugging via UART  

* __BME280 Sensor__ – Measures temperature, humidity, and pressure

* __MQ135 Sensor__ – Detects air quality (gas concentration)  

* __Sound Sensor__ – Used for triggering  
  
* __LED__ – Visual indicator for interrupt status  

* __Resistor (330 ohm)__ – Current limiting resistor for the LED  

* __Active Buzzer__ – Audible alert when environment is unhealthy  

* __Jumper Cables__ – Used to connect components  

* __Breadboard__ – For prototyping and sensor wiring

* __USB Cable / Power Supply__ – Powering the STM32 and ESP32 modules

# Software Components

* __STM32CubeIDE__ – Embedded firmware development for STM32

* __Arduino IDE__ – Programming  the ESP32 module

* __C/C++ Programming Languages__ – Core programming languages for embedded development

* __Adafruit IO Dashboard__ – Visualization of environmental sensor data  

* __Termitte Terminal__ – Serial debugging and log monitoring via UART 

* __FreeRTOS (CMSIS-RTOS v2)__ – Real-time task management on STM32

* __Digital Filtering (Moving Average)__ – Smoothing sensor readings 

* __MQTT Protocol__ – Data transmission from ESP32 to Adafruit IO  

* __GPIO / ADC / UART Communication Peripherals__ – Use of STM32 hardware features

# Pin Configuration

Figure 3 : Pin Configuration In The STM32CubeIDE

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/configuration.png" alt="Configuration" width="500"/>

PA0 -> ADC_IN0 -> A0 Pin MQ135 Gas Sensor

PA9 -> USART1_TX -> ESP32 - U2_RXD (G16)

PA10 -> USART1_RX -> ESP32 - U2_TXD (G17)

PB6 -> I2C1_SCL -> BME280 - SCL

PB7 -> I2C1_SDA -> BME280 - SDA

PC2 -> GPIO_Output -> LED

PC3 -> GPIO_EXTI3 -> Sound Sensor

STM32F0DISC -> 3V -> Board

STM32F0DISC -> GND -> Board

ESP32 -> GND -> Board

ESP32 -> G2 -> Active Buzzer

All the GNDs are connected.

# Challenges Encountered And Solutions

1) A driver sourced from GitHub for the BME280 sensor was not suitable for use with an RTOS. It relied on blocking methods. Therefore, it was adapted to be RTOS-compatible by introducing mutex protection and modifying the functions to use non-blocking behavior.

2) The gas sensor produced unstable readings. To improve stability and reliability, a moving average filter was applied.

3) At first, individual tasks were assigned to each sensor. This design caused a RAM overflow because of the system’s constrained memory. The solution was to consolidate all sensor operations into a single task, calling separate functions for each sensor within it.

4) Using float variables caused a RAM overflow in the system. To solve this, I read sensor values as integers, performed scaled mathematical operations, and only converted them to float when needed.

5) Because of STM32's hardware constraints, the evaluation logic and buzzer control were handled by the ESP32.

# What I Learned from This Project

* I learned how to synchronize RTOS structures (tasks, semaphores, mutexes, queues) effectively within a real embedded system project.

* I gained practical experience adapting and developing RTOS-compatible drivers, refactoring blocking code into non-blocking implementations.

* I understood the critical importance of synchronizing hardware peripherals like GPIO, ADC, and UART within an RTOS environment.

* I experienced task management and memory optimization under resource-constrained conditions, such as avoiding RAM overflow.

* I realized the significance of digital filtering techniques (e.g., moving average filter) to produce stable sensor data when working with multiple sensors.

* I successfully enabled synchronized communication between two different microcontrollers (STM32 and ESP32) over UART.

* I learned about JSON data formatting, string handling with functions like snprintf, and protecting UART communication using mutexes.

* I implemented data transmission using the MQTT protocol and integrated real-time sensor visualization on an IoT dashboard.

* Through hands-on work, I observed the benefits of event-driven software architecture, such as lower CPU usage and reactive task execution.

* I developed the ability to analyze and solve problems encountered during the project, including RAM overflow, incompatible drivers, and noisy sensor data.

* Beyond technical problem solving, I cultivated discipline in project documentation, visual presentation, and maintaining a shareable, professional-quality repository.

# Known Limitations

* Sensor readings depend on environmental placement and may vary due to airflow or proximity.

* No persistent data logging was implemented.

* Power efficiency was not optimized (system always active).

* ESP32-side decision logic may require further tuning for real-world deployment.

# Future Improvements

* Add persistent SD card logging on STM32 or ESP32.

* Include OTA (Over-the-Air) update capability on ESP32.

* Integrate an OLED display to show local sensor data.

* Implement ESP32 deep sleep for power saving.

* Add configuration mode for threshold values via dashboard.
