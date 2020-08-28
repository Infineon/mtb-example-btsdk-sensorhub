# CE228683 - Sensor Hub

This example demonstrates multiple sensors (motion sensor, 3D magnetic sensor, and thermistor) interfaced with Cypress CYW208xx and CYW207xx Bluetooth SoCs using ModusToolbox™ software.

**Note:** This code example can also be used with PSoC 6 BLE kits programmed with the [PSoC 6 BLE Wi-Fi Gateway code example](https://github.com/cypresssemiconductorco/mtb-example-psoc6-ble-wifi-gateway). The PSoC 6 BLE Wi-Fi gateway collects sensors data over GATT notification, displays it on the TFT screen, and publishes it to the cloud.

## Requirements

- [ModusToolbox™ software](https://www.cypress.com/products/modustoolbox-software-environment) v2.1

- Programming Language: C

- Associated Parts:
   - [CYW20819](https://www.cypress.com/datasheet/CYW20819)
   - [CYW20820](https://www.cypress.com/datasheet/CYW20820)
   - [CYW20719](https://www.cypress.com/documentation/datasheets/cyw20719-enhanced-low-power-bredrble-bluetooth-50-soc)
   - [CYW20735](https://www.cypress.com/documentation/datasheets/cyw20735b1-single-chip-bluetooth-transceiver-wireless-input-devices)

## Supported Kits
-  [CYW920819EVB-02 Evaluation Kit](http://www.cypress.com/CYW920819EVB-02) - Default target
-  [CYW920820EVB-02 Evaluation kit](http://www.cypress.com/CYW920820EVB-02)
-  [CYW920719B2Q40EVB-01 Evaluation kit](https://community.cypress.com/docs/DOC-17736)
-  [CYW920735Q60EVB-01 Evaluation kit](http://www.cypress.com/CYW920735Q60EVB-01)
-  [S2GO 3D SENSE TLV493D](https://www.infineon.com/cms/en/product/evaluation-boards/s2go_3d-sense_tlv493d/)
-  [My IoT Adapter](https://www.infineon.com/cms/en/product/evaluation-boards/my-iot-adapter/)
-  [Rotate Knob 3D 2 Go Kit](https://www.infineon.com/cms/en/product/evaluation-boards/rotate-knob-3d-2-go-kit/)

## Hardware Setup

This example uses the board's default configuration. Refer to the kit user guide to ensure that the board is configured correctly.

The CYW20819 and CYW20820 base boards come with motion sensor and thermistor. The 3D Magnetic Sensor shield board needs to be plugged into the Arduino connector of the base boards through the My IoT adapter board.

**Figure 1. Board Setup**
![Board Setup](images/board_setup.png)

## Software Setup

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://ttssh2.osdn.jp/index.html.en). All other required software come bundled with the Eclipse IDE for ModusToolbox.

To use a Windows PC as the BLE Central device for the GATT Client application, install [CySmart Host Emulation Tool](https://www.cypress.com/documentation/software-and-drivers/cysmart-bluetooth-le-test-and-debug-tool). You will also need [CY5677 CySmart BLE 4.2 USB Dongle](http://www.cypress.com/documentation/development-kitsboards/cy5677-cysmart-bluetooth-low-energy-ble-42-usb-dongle).

To use an iOS or Android smartphone as the BLE Central device, download the CySmart app. Scan the following QR codes from your mobile phone to download the CySmart app.

![AppQR](images/qr_code.png)

This example requires no additional software or tools.

## Using the Code Example

### In Eclipse IDE for ModusToolbox:

1. Click the **New Application** link in the Quick Panel (or, use **File** > **New** > **ModusToolbox Application**).

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog and click **Next**.

3. In the **Project Creator - Select Application** dialog, select **wiced\_btsdk**.

   This project contains the SDK. It is used by all BT-SDK applications. You will need to create this project just once in the working directory (i.e., Eclipse workspace). Ignore if you have already created this project.

    **Note:** Do not rename this project. All BT-SDK apps use this project name in application Makefiles.

4. After the 'wiced\_btsdk' project is created, choose the **Sensor_Hub** application from the same **Project Creator - Select Application** dialog.

5. Optionally, update the **Application Name** field with the application name.

6. Click **Create** to complete the application creation process.

    **Note:** Both the applications are created for the board that you have selected in Step 2.

7. Select the *GATT_server* application in the IDE. In the Quick Panel, click the **Build** link to build the application.

8. Connect the first board to your PC.

9. To program it (download the application), select **Program** in the Quick Panel.

    **Note:** If the download fails, it is possible that a previously loaded application is preventing programming.

    For example, the application may use a custom baud rate that the download process does not detect or the device may be in low-power mode. In that case, it may be necessary to put the board in recovery mode, and then try the programming operation again from the IDE. To enter recovery mode, first, press and hold the Recover button (SW1), press and release the Reset button (SW2), and then release the Recover button (SW1).

10. Unplug the board from PC.

11. Select the *Sensor_Hub* application, and in the Quick Panel, select **Build** to build the application.

12. Connect the other board and select **Program** in the Quick Panel. After successfully programming the second app, plug in the first board which has the BLE Sensor Hub application.

### In Command-line Interface (CLI):

1. In Windows, run Cygwin by clicking on the *Cygwin.bat* file from the  *<install_folder>\ModusToolbox\tools_2.x\modus-shell* folder.

   The default install folder is the user's home directory. All the following steps should be run from inside the Cygwin window.

2. Go to the directory that you want to use for your workspace. Use the `mkdir <directory>` command to create the new directory and run the `cd <directory>` command.

3. Clone the *wiced\_btsdk* repo first. As mentioned earlier, this project contains the SDK used by all apps. You will need to create this project just once in the working directory. For example:
   ```
   > git clone https://github.com/cypresssemiconductorco/wiced_btsdk
   ```
4. Clone the app repo *mtb-example-btsdk-ble-sensorhub*. The application repo directory should be at the same folder level as *wiced_btsdk*. For example:
   ```
   > git clone https://github.com/cypresssemiconductorco/mtb-example-btsdk-ble-sensorhub
   ```

5. The *wiced_btsdk* repo contains references to other repos. To download all the required collateral, navigate to the *wiced_btsdk* folder and use the `make getlibs` command. For example:
   ```
   > cd wiced_btsdk
   > make getlibs
   ```
6. Navigate into the *Sensor_Hub* application folder from the *wiced_btsdk* folder. To build the application using the default target (CYW920819EVB-02), run the `make build` command:
   ```
   > cd ../mtb-example-btsdk-ble-sensorhub
   > make build
   ```
    To build the application with a different target (supported by the application), use the following command:
    ```
    > cd ../mtb-example-btsdk-ble-sensorhub
    > make build TARGET=<BSP>
    ```
    Example:
    ```
    > cd ../mtb-example-btsdk-ble-sensorhub
    > make build TARGET=CYW920820EVB-02
    ```
7. Connect the first board to your PC.

8. To program (download the application) the board, run the `make qprogram` (for default target - CYW920819EVB-02)
   ```
   > make qprogram
   ```
   If you have built the application for a different target, use the following command to program:
    ```
    > cd ../mtb-example-btsdk-ble-sensorhub
    > make qprogram TARGET=<BSP>
    ```
    Example:
    ```
    > cd ../mtb-example-btsdk-ble-sensorhub
    > make qprogram TARGET=CYW920820EVB-02
    ```
   **Note:** If the download fails, it is possible that a previously loaded application is preventing programming. For example,the application may use a custom baud rate that the download process does not detect or the device may be in low-power   mode. In that case, it may be necessary to put the board in recovery mode, and then try the programming operation again from the IDE. To enter recovery mode, first, press and hold the Recover button (SW1), press and release the Reset button (SW2), and then release the Recover button (SW1).

### In Third-party IDEs:

1. Follow the instructions from the [CLI](#in-command-line-interface-cli) section to download or clone the application and wiced_btsdk repositories, and import libraries using the `make getlibs` command.

2. Export the application to a supported IDE using the `make <ide>` command from the "Sensor_Hub* folder.

3. Follow the instructions displayed in the terminal to create or import both the applications as an IDE project.

For more details, see the "Exporting to IDEs" section of the ModusToolbox User Guide: *{ModusToolbox install directory}/ide_{version}/docs/mtb_user_guide.pdf*.

## Operation

### Using with CySmart Desktop Application as BLE Central:

1. Connect the board to your PC using the provided USB cable through the USB connector.

2. Open a terminal program and select the WICED PUART COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board with the *Sensor_Hub* application.

   After programming, the application starts automatically. The Sensor Hub will start advertising.

   **Figure 2. Terminal Output for Sensor Hub During Advertising**
   ![Sensor Hub Adv](images/sensor_hub_adv.png)

4. Open the [CySmart desktop application](https://www.cypress.com/documentation/software-and-drivers/cysmart-bluetooth-le-test-and-debug-tool) and connect to the [CySmart CY5677 dongle](http://www.cypress.com/cy5677) (Central device).

   See the [CySmart user guide](https://www.cypress.com/file/232316/download) to learn how to use the desktop application.

5. Using the CySmart desktop application, **scan** and **connect** to the 'Sens Hub' device.

6. If prompted, click **Yes** to update the connection parameters.

7. Go to the **Device** tab and click **Discover all attributes**.

8. Click **Enable all Notifications**.

   The Sensors values are displated on the UART terminal.

9. Click **Disable All Notifications** to stop reading the sensor values.

10. Click **Disconnect** to disconnect from the Central device.


### Using CySmart iOS/Android App on Smartphone as BLE Central

1. Connect the board using the provided USB cable through the USB connector.

2. Open any serial terminal and select WICED PUART COM. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board with the *Sensor_Hub* application.

4. Turn ON Bluetooth on your Android or iOS device and launch the CySmart app.

5. Swipe down on the CySmart app home screen to start scanning for BLE Peripherals; your device appears in the CySmart app home screen with the name 'Sens Hub'`. Select your device to establish a BLE connection (see Figure 3).

6. Select **GATT DB** from the carousel view. Swipe left or right to change carousel selections.

7. Select **Unknown Service** and then select the Characteristic with the **Notify** property.

8. Select **Notify**. The device will start sending GATT notifications to the mobile.
   **Figure 3. CySmart App**
   ![CySmart App](images/cysmart_guide.png)

   Sensor values will be displayed on the UART terminal as follows.

   **Figure 4. Sensor Values**
   ![Sensor Values](./images/sensor_hub_values.png)

## Design and Implementation

### Introduction

This code example implements a GATT Server, GAP Peripheral role , ADC and I2C on the CYW208xx/CYW207xx device. Once the device is powered ON, it boots up and does the following things:

1. Initializes the Bluetooth stack
2. Initializes PUART for input
3. Initializes I2C and ADC
4. Registers a button interrupt
5. Initializes the notification timer for all sensors
6. Registers the GATT database and GATT events callback
7. Initializes all sensors
8. Starts undirected or directed advertisements based on the bond data present

You can now connect to the device using a GAP Central device. Upon connection, the device will request connection parameters to be updated (specifically, the connection interval to 100 ms). If the request is accepted, the connection interval changes to 100 ms.

The GAP Central can now discover all attributes and enable GATT notifications. The Peripheral will start sending sensor values with the defined interval. Moving the board or rotating the knobs changes the values in the console output.

The GATT Server implements a custom service with a custom characteristic. This characteristic is readable and notifiable.

Application-level source files for this code example are listed in the following table:

|**File Name**|**Description**|
|-----------------------------------|-------------------------------------------------------|
| *sensor_hub.c* | Entry to the application. It initializes the PUART for debugging. |
| *sensor_hub_ble.c* | Handles BLE initialization, configuration, advertisement, notifications, and responses to BLE events. It also manages the timers, button, and interrupt callbacks.|
| *motion_sensor_hw.c* |Handles hardware configuration for motion sensors.|
| *mag3d_sensor_hw.c* |Handles the hardware for the 3D magnetic sensor.|
| *temp_sensor_hw.c* |Handles the hardware and ADC measurements for reading the temperature.|
| *app_bt_cfg.c* | Runtime Bluetooth stack configuration parameters.|
| *Lsm9ds1_reg.c* |Contains device drivers for the LSM9DS1 motion sensor.|

**Figure 11. Application Flowchart for Sensor Hub**
![Sensor Hub Flow](images/sensor_hub_flow.png)

## Resources and Settings
This section explains the ModusToolbox resources and their configuration as used in this code example. Note that all the configuration explained in this section has already been done in the code example. Eclipse IDE for ModusToolbox stores the configuration settings of the application in the *design.modus* file. This file is used by the graphical configurators, which generate the configuration firmware. This firmware is stored in the application’s *GeneratedSource* folder.

* **Device Configurator:** Use this tool to enable/configure the peripherals and pins used in the application. See the
[Device Configurator Guide](https://www.cypress.com/ModusToolboxDeviceConfig).

* **Bluetooth Configurator:** Use this tool to generate/modify the BLE GATT database. See the
[Bluetooth Configurator Guide](https://www.cypress.com/ModusToolboxBLEConfig).

## Related Resources
| Application Notes||
|--|--|
| [AN225684](http://www.cypress.com/an225684):  Getting Started with CYW208xx | Describes CYW208xx device and how to build your first ModusToolbox project |
|**Code Examples**| Visit the [Cypress GitHub repo](https://www.cypress.com/mtb-github) for a comprehensive collection of code examples using Eclipse IDE for ModusToolbox|
|**Device Documentation**|
|[CYW20819 Device Datasheet](https://www.cypress.com/datasheet/CYW20819)|
|[CYW20820 Device Datasheet](https://www.cypress.com/datasheet/CYW20820)|
|[CYW20719 Device Datasheet](https://www.cypress.com/documentation/datasheets/cyw20719-enhanced-low-power-bredrble-bluetooth-50-soc)|
|[CYW20735 Device Datasheet](https://www.cypress.com/documentation/datasheets/cyw20735b1-single-chip-bluetooth-transceiver-wireless-input-devices)|
|[TLV493DA1B6 Datasheet](https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/3d-magnetics/tlv493d-a1b6/)|
|**Development Kit Documentation**|
|[CYW920819EVB-02 Evaluation Kit](http://www.cypress.com/CYW920819EVB-02)|
|[CYW920820EVB-02 Evaluation Kit](http://www.cypress.com/CYW920820EVB-02)|
|[CYW920719B2Q40EVB-01 Evaluation kit](https://community.cypress.com/docs/DOC-17736)|
|[CYW920735Q60EVB-01 Evaluation kit](http://www.cypress.com/CYW920735Q60EVB-01)|
|[S2GO_3D-SENSE_TLV493D Evaluation Board](https://www.infineon.com/cms/en/product/evaluation-boards/s2go_3d-sense_tlv493d/)|
|**Tool Documentation**|
|[Eclipse IDE for ModusToolbox](https://www.cypress.com/modustoolbox)    | The cross-platform, Eclipse-based IDE for IoT designers that supports application configuration and development targeting converged MCU and wireless systems.             |

--------------------------------------------------------------------------------------------
## Document History

Document Title: *CE228683* - *Sensor Hub*

| Version | Description of Change |
| ------- | --------------------- |
| 1.0.0   | New code example      |
| 1.1.0   | Reconnection issue resolved      |
| 1.2.0   | New features added with multiple sensor      |

------

![Banner](images/footer_banner.png)

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2020. This document is the property of Cypress Semiconductor Corporation and its subsidiaries (“Cypress”).  This document, including any software or firmware included or referenced in this document (“Software”), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product.  CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, “Security Breach”).  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications.  To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document.  Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  “High-Risk Device” means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  “Critical Component” means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  You shall indemnify and hold Cypress, its directors, officers, employees, agents, affiliates, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
Cypress, the Cypress logo, Spansion, the Spansion logo, and combinations thereof, WICED, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress in the United States and other countries. For a more complete list of Cypress trademarks, visit cypress.com.  Other names and brands may be claimed as property of their respective owners.