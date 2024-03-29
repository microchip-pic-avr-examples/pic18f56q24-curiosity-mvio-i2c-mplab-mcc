<!-- Please do not change this logo with link -->

[![MCHP](images/microchip.png)](https://www.microchip.com)

# Multi-Voltage I/O (MVIO) Example Using I2C to Communicate with an EEPROM 3 Click board and MCP9800 Using the PIC18F56Q24 Microcontroller Generated with MCC Melody

<!-- This is where the introduction to the example goes, including mentioning the peripherals used -->
This example shows how the Multi-Voltage I/O (MVIO) module can be used to interface components that operate at a voltage different than the main device operating voltage without any external hardware. In this example, the MCP9800 Temperature Sensor was interfaced using standard voltage domain since it operates at the same voltage as the microcontroller, and the EEPROM 3 Click board® was interfaced using the MVIO module since it operates at a voltage different than the main VDD of the microcontroller. For this demonstration, the temperature is measured using the MCP9800 sensor and the data is then stored to the  EEPROM 3 Click board® both using the I<sup>2</sup>C protocol.
## Related Documentation

- [PIC18F-Q24 Family Product Page](https://www.microchip.com/en-us/product/PIC18F56Q24)
- [PIC18F56Q24 Data Sheet](https://www.microchip.com/DS40002503)
- [PIC18F56Q24 Getting Started with MVIO Code example](https://bitbucket.microchip.com/projects/EBE/repos/pic18f56q24-getting-started-with-mvio-mplab-mcc/browse)
## Software Used

- [MPLAB® X IDE](http://www.microchip.com/mplab/mplab-x-ide) v6.15 or newer
- [MPLAB XC8](http://www.microchip.com/mplab/compilers) v2.45 or newer
- [PIC18F-Q_DFP Series Device Pack](https://packs.download.microchip.com) v1.23.425 or newer
- [MPLAB Code Configurator](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator) 5.5.7 or newer
- [MPLAB Code Configurator Melody](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator/melody) core 2.6.2 or newer


## Hardware Used
- [PIC18F56Q24 Curiosity Nano](https://www.microchip.com/en-us/development-tool/ev01e86a) 

<img src="images/56Q24-Board.png" width = "600"><br>

- [PICkit Serial I2C Demo Board](https://www.microchip.com/en-us/development-tool/PKSERIAL-I2C1) MCP9800 sensor on this board is used for the temperature sensor

<img src="images/i2c_demo_board.png" width = "600"><br>

- [EEPROM 3 Click](https://www.mikroe.com/eeprom-3-click) from MIKROE

<img src="images/eeprom-3-click.png" width = "600"><br>

- [Saleae Logic Analyzer](https://www.saleae.com/)

Optional:
 - [Base board for Click boards](https://www.microchip.com/en-us/development-tool/AC164162)

 <img src="images/baseboard_for_curiosity.png" width = "600"><br>

## Prerequisites

For the MVIO to work together with the Curiosity Nano board, disconnect the R201 resistor connecting V<sub>DDIO2</sub> to VCC_TARGET.

<img src="images/56Q24-MVIO-BOARD.png" width = "300"><br>

A power supply must be connected to the V<sub>DDIO2</sub> pin. There are two possible use cases:
<br />
**a.** Connecting an external DC power supply to the V<sub>DDIO2</sub> pin and GND.
<br />
**b.** Together with the Curiosity Nano, use the V<sub>BUS</sub> as power supply, by connecting a wire between the V<sub>BUS</sub> and the V<sub>DDIO2</sub>.

I<sup>2</sup>C Connections
- MCP9800 Temperature Sensor
<br><img src="images/wiring_diagram.png" width = "600"><br>
- EEPROM 3 Click Board
<br><img src="images/i2c_connections.png" width = "600"><br>
**NOTE:** Jumper setting on the EEPROM 3 Click Board has been changed to supply 5V and a wire between V<sub>DDIO2</sub> and V<sub>BUS</sub> (5V). See above.

## Initial Setup

1. In the **Project Resources** window: check the dropdown box for **System**, then click Clock Control.
   
<br><img src= "images/project_resources_clock_control.png">

 2. In the Clock Control window: go to the **Easy View** tab, and set **Clock Settings** to **HFINTOSC**. 
  - Clock Source: HFINTOSC
  - Internal Clock: 4 MHz
  - Clock Divider: 4

<br><img src="images/clock_control.png">

3. In the **Project Resources** window: check the dropdown box for **System**, then  click Configuration Bits.
   
<br><img src= "images/project_resources_configuration_bits.png">

 4. In the Configuration Bits window: go to the **Easy View** tab, and use dropdown boxes to set the **External Oscillator Selection** to **Oscillator not enabled** & **Reset Oscillator Selection** to **HFINTOSC w HFFRQ = 4 MHz and CDIV = 4:1**
  - External Oscillator: Disabled
  - Reset Oscillator: 4 MHz & Clock Divider 4:1

<br><img src="images/config1.png">

5. In the **Project Resources** window: check the dropdown box for **System**, then check the dropdown box for **MVIO** click V<sub>DDIO2</sub>.

<br><img src="images/project_resources_mvio.png">

These are the settings for this example. 
- I/O Monitor: Enable
- Hysteresis: Enable
- LVD Trip Point: Disabled

<br><img src="images/vddio2_easy_view.png">

## Setup

In the left-hand pane, under **Device Resources**:
   1. Open the **Drivers** dropdown.
   2. Click the green plus sign next to the **I2C_Host** peripheral to add them to the project.

<br><img src="images/add_i2c.png">

In the **Project Resources** window: Click the dropdown box for Drivers then I<sup>2</sup>C, then click **I2C1_Host**.

<br><img src="images/project_resources_i2c.png">

In the I2C_Host window: go to the **Easy View** tab, and use the dropdown box to select **I2C1**.

<br><img src="images/select_i2c1.png">

After these settings, the window will display the following:

<br><img src="images/i2c_host_easy_view.png">
   1. Open the **Drivers** dropdown.
   2. Click the green plus sign next to the **I2C_Host** peripheral to add them to the project.

<br><img src="images/add_i2c.png">

In the **Project Resources** window: Click the dropdown box for Drivers, then I<sup>2</sup>C, then click **I2C2_Host**.

<br><img src="images/project_resources_i2c1.png">

In the I2C_Host window: go to the **Easy View** tab, verify the setting match the picture below.

<br><img src="images/i2c2_host_easy_view.png">

In the Pin Grid View window: For I2C1 click on **RC4** for **SCL1** & **RC3** for **SDA1**. For I2C2 click on **RB1** for **SCL2** & **RB2** for **SDA2**. See below. **Note:** Brown shaded MVIO pins are powered by the V<sub>DDIO2</sub> voltage domain.


<br><img src="images/pin_grid_view1.png">

In the **Project Resources** window: check the dropdown box for **System**, then click **Pins**.

<br><img src="images/project_resources_pins.png">

The **Pins** Tab shows up in MPLAB on the right side: select Start High for all I<sup>2</sup>C pins.

<br><img src="images/pins_i2c1.png">

## Operation
The example code starts off by communicating with the MCP9800 temperature sensor which operates at the same voltage as the main device V<sub>DD</sub> of 3.3V using the I2C2 module. The I2C2 module is configured to send the appropriate I<sup>2</sup>C command to configure/initialzie the MPC9800, and then it  sends an I<sup>2</sup>C command to read the Temperature Sensor. All of this happens on the V<sub>DD</sub> voltage domain (3.3V). After the temperature has been read by the device, the next step is to store the measured value onto the external serial EEPROM device. The EEPROM device used in this example operates at 5V instead of the 3.3V that the microcontroller and temperature sensor operate at. The MVIO module was used to communicate with the external serial EEPROM device; 5V was supplied to the V<sub>DDIO2</sub> supply pin and the I2C1 module was configured to operate on the V<sub>DDIO2</sub> voltage domain. The I2C1 module was used to write the measured temperature data to the EEPROM3 Click board, and then read the data back to verify it was written successfully. The MVIO module handles all of the voltage level shifting needed to communicate with a 5V device when the microcontroller is operating at 3.3V without the need for any external circuitry. 

Saleae. Capture of the I<sup>2</sup>C transactions

 - MCP9800 I<sup>2</sup>C Command for Configuration / Initialization of Temperature Sensor (V<sub>DD</sub> voltage domain - 3.3V)
 <br><img src="images/saleae_MCP9800_Config.png"><br>
 
 - MCP9800 I<sup>2</sup>C Command to read value from Temperature Sensor (V<sub>DD</sub> voltage domain - 3.3V)

<br><img src="images/saleae_mcp9800_read.png"><br>

 - EEPROM 3 Click I<sup>2</sup>C Write command to store measured temperature data onto external serial EEPROM device (V<sub>DDIO2</sub> voltage domain - 5V)
<br><img src="images/saleae_eeprom_write.png"><br>

 - EEPROM 3 Click I<sup>2</sup>C Read command to verify data was stored correctly (V<sub>DDIO2</sub> voltage domain - 5V)
<br><img src="images/saleae_eeprom_read.png"><br>

## Summary
This example demonstrates how the MVIO module can be used to quickly and easily communicate with a device that operates at an entirely diffent voltage level than that of the main microcontroller. The microcontroller's main V<sub>DD</sub> voltage domain was used to read from a sensor that operated at the same voltage, and the MVIO module was used to drive a subset of pins and power one of the available I2C modules at a different voltage (V<sub>DDIO2</sub>) in order to communicate with an external serial EEPROM device that operated at 5V. The MVIO module handles all of the voltage and logic level shifting internally, and removes the need for any external circuitry or components. Everything that was done in this example can be seen in the Saleae Logic Analyzer traces / weaveforms above, and it is clearly demonstrated that the device is capable of driving it's I/Os at two different voltage levels using the MVIO module.

