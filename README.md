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

Figure 1 : System Overview

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/system_overview.jpeg" alt="System Overview" width="500"/>

Figure 2 : Adafruit IO Dashboard

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/dashboard.png" alt="System Overview" width="500"/>

# Hardware Components

* __STM32F0DISC__

* __ESP32 WROOM 32D__

* __USB to TTL (Serial) Communication Module (for debug)__

* __BME280 Sensor__

* __MQ135 Sensor__

* __Sound Sensor__
  
* __LED__

* __Resistor (330 ohm)__

* __Active Buzzer__

* __Jumper Cables__

# Software Components

* __STM32CubeIDE__

* __Arduino IDE (for the ESP32)__

* __C/C++ Programming Languages__

* __Adafruit IO Dashboard__

* __Termitte Terminal (for debug)__

* __UART Serial Communication__

* __FreeRTOS (CMSIS-RTOS v2)__

* __Digital Filtering (Moving Average)__

* __MQTT Protocol__

* __GPIO / PWM / ADC Peripherals__




