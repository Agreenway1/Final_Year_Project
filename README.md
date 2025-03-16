# **Running the Remote Palpation Robotic System**

## **How to Run the Code**

### **1. Setup Hardware**
- **Connect the TI C2000 board** to the **PC** (via USB/UART) and **Arduino UNOs**.
- **Power the CNC rail system** and **BLDC motor**.
- **Ensure all sensors (Hall, ADC) are connected properly.**

### **2. Compile and Upload Code**
#### **2.1 TI C2000 Compilation**
- Use **Code Composer Studio (CCS)** to compile and upload the **TI C2000 code**.
- Ensure the correct **UART and GPIO pins** are configured.

#### **2.2 Arduino Code Compilation**
- **Use Arduino IDE** to upload the following:
  - **`stepper_control_code.ino`** → Stepper Arduino.
  - **`bldc_foc_hall.ino`** → BLDC Arduino.
  - **`temperature_monitor.ino`** → Temperature Arduino.

### **3. Running the System**
- **Start the GUI** → Select **pathology and position**.
- **The system will automatically:**
  1. **Update heater temperatures** based on pathology.
  2. **Move the plunger to the correct XY position**.
  3. **Apply the correct force using the BLDC motor.**
  4. **Send real-time heater temperature data to the monitoring Arduino.**
- **Monitor outputs using Serial Monitor (Arduino IDE) or CCS.**

---

## **Pending Arduino Code Development**
🚨 **Two additional Arduino programs still need to be developed:**
1. **Heating Control PWM Arduino**
   - This Arduino will **send PWM signals to the MOSFET driver** to control heating elements.
2. **MOSFET Voltage Sensing Arduino**
   - This Arduino will **read and estimate voltage across the MOSFETs** and stream data back to the TI C2000.
   - The **heating temperature control system needs to be updated** to reflect this feedback.

These additional Arduinos will improve **heater control accuracy and closed-loop feedback.**

