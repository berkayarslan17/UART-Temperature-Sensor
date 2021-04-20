# UART-Temperature-Sensor
The aim of this project is to design a UART temperature sensor device. It will be aimed to make an algorithm that will display the temperature data when the “AT+TEMP?” is written.
For this project, it will be used STM32G031K8 and KY013 NTC Thermistor and STM32CubeIDE as a workspace.

## Schematic Diagram
![schematic](https://user-images.githubusercontent.com/44584158/115393020-e49edb00-a1e9-11eb-9926-ad0bcd326e3d.jpg)

## Parts List
### KY-013 Temperature-Sensor
![resim](https://user-images.githubusercontent.com/44584158/115393162-1021c580-a1ea-11eb-8d54-1c025b12c143.png)

### STM32G031K8
![resim](https://user-images.githubusercontent.com/44584158/115393235-22036880-a1ea-11eb-8434-fd0c5b136994.png)

## Project Setup
![resim](https://user-images.githubusercontent.com/44584158/115393304-35163880-a1ea-11eb-9198-30e29b0a455a.png)

## Detailed Task List
Task 1. Determine the resister values for voltage divider. (?)

Task 2. Configure the ADC and retrieve sample with 10Hz. (✓)

Task 3. Convert ADC result to temperature. (✓)

Task 4. Use low pass filter to filter temperature results. (✓)

Task 5. Save the data to queue with using LIFO method. (✓)

Task 6. Configure the UART and write a listener that sends queued data on request. (✓)

## Conclusion
It sends the queued data on request.
![resim](https://user-images.githubusercontent.com/44584158/115393381-4bbc8f80-a1ea-11eb-8791-d38ad1e01b23.png)


## References
[1] https://datasheetspdf.com/pdf-file/1402026/Joy-IT/KY-013/1 (KY-013 NTC Thermistor datasheet)

[2] https://www.st.com/resource/en/reference_manual/dm00371828-stm32g0x1-advanced-armbased-32bit-
mcus-stmicroelectronics.pdf (STM32g0 reference manual)

[3] https://github.com/fcayci/stm32g0
