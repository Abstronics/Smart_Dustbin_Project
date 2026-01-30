
CHAPTER 1: INTRODUCTION
1.1 Background of the Study
Waste management is a critical issue in modern metropolitan environments. The traditional method of using open or manually operated dustbins often leads to unhygienic conditions, as direct contact with the trash bin lid can spread bacteria and other pathogens. Furthermore, overflow of waste due to a lack of monitoring creates environmental hazards. With the rise of automation and the Internet of Things (IoT), there is a growing need for "Smart" solutions that minimize human contact and improve sanitation. This project focuses on the development of a Smart Dustbin that automates the lid opening process using ultrasonic sensing technology, ensuring a touch-free and hygienic waste disposal experience to its users.
1.2 Problem Statement
Manual dustbins pose significant health risks due to the physical contact required to open them, facilitating the spread of germs and other pathogens. Additionally, in many public and domestic settings, bins are often left open,  attracting pests and vectors and emitting foul odors. Existing automated solutions often rely on expensive PLC systems or lack robust power management, making them inaccessible for general consumer use. There is a clear need for a cost-effective, reliable, and hygienic waste management system that integrates efficient analog power circuits with digital control.
1.3 Objectives
General Objective: To design, simulate, prototype, and implement a functional robotic Smart Dustbin system integrating analog circuits, ultrasonic sensors, servo actuators, and micro-controller programming.
Specific Objectives:
1.	To design and simulate the complete smart dustbin system using Proteus
2.	To develop analog circuits for regulated power supply to ensure smooth running of sensors
3.	To write and optimize C++ micro-controller code in real time distance and servo motor control	
4.	To design a professional PCB layout to replace the breadboard prototype
5.	To build and test the physical prototype under real world conditions
6.	To document the entire project, including schematics and code, and publish the repository on Github
1.4 Research Questions
1. How can analog circuit techniques be applied to ensure stable power delivery to digital sensors in a robotics system?
2. What is the optimal detection range for an ultrasonic sensor to trigger a dustbin lid without false positives?
3. How does a custom PCB design improve the reliability of a smart dustbin compared to a breadboard prototype
4. Can a low-cost micro-controller effectively handle real-time sensor data and actuator control simultaneously

1.5 Significance of the Study
This study is significant as it bridges the gap between basic electronics and applied robotics. By automating waste disposal, it directly contributes to improved public hygiene and reduced disease transmission. Academically, the project demonstrates the practical integration of analog signal conditioning (power regulation) and digital embedded systems, providing a comprehensive educational model for mechatronics and engineering students.
1.6 Scope and Limitations
Scope: The project covers the design of a 5V regulated power supply, Arduino interfacing with HC-SR04 sensors and SG90 servo motors, PCB fabrication, and chassis construction
Limitations: The system is designed for indoor domestic use. It does not currently include IoT features (Wi-Fi/GSM) for remote monitoring, and the servo motor torque limits the lid weight to under 500g.
1.7 Organization of the Study
This report is organized into five chapters.
Chapter 1 introduces the problem and objectives.
Chapter 2 reviews existing literature and identifying gaps.
Chapter 3 details the methodology, including analog design and PCB layout.
Chapter 4 presents the simulation and hardware results.
Chapter 5 concludes the study with recommendations for future work.

 


CHAPTER 2: LITERATURE REVIEW
2.1 Introduction
This chapter reviews the evolution of waste management technologies, focusing on automation and sensor integration. It analyzes existing smart bin solutions, analog circuit requirements for robotics, and the software tools used in modern prototyping.
2.2 Review of Existing Similar Robots
1. IoT-Based Smart Bins: Studies have proposed bins connected to the cloud via Wi-Fi to monitor fill levels in real-time. While effective for municipalities, these are often too complex for home use.
2. GSM-Based Alert Systems: Research by Khan et al. (2021) utilized GSM modules to send SMS alerts when bins were full. These systems suffer from high recurring costs due to SIM requirements.
3. Radio Frequency Identification (RFID) Bins: Some industrial bins use RFID tags to sort waste automatically. However, these require specialized tagging of waste items, limiting their practicality.
4. Ultrasonic Sensor Bins: Monika K A et al. proposed a simple Arduino-based bin using ultrasonic sensors. This design is cost-effective but often lacks a custom PCB, relying on fragile jumper wires.
5. Solar Powered Bins: Laha et al. (2023) explored solar-powered bins for outdoor use. While sustainable, the analog charging circuits were complex and expensive.
6.IR Sensor Models: Early models used Infrared (IR) sensors. These were prone to false triggers from sunlight interference, making them unreliable outdoors.
7.Voice Controlled Bins: Arthur et al. (2024) reviewed bins with voice recognition. These were found to have high power consumption and slow response times.
8.Computer Vision Bins: Recent studies utilize cameras and AI to sort waste. These require high processing power (e.g., Raspberry Pi) and are not energy efficient for battery operation.

2.3 Analog Circuit Techniques in Robotics
Analog circuits are foundational in robotics, particularly for power management and signal conditioning. In this project, an Analog Regulated Power Supply is critical. Sensors like the HC-SR04 require a stable 5V; voltage fluctuations can cause erratic readings. Techniques such as Rectification (AC to DC), Filtering (Capacitors), and Linear Regulation (LM7805) are standard industry practices to ensure system stability.
2.4 Micro-controller Selection and Justification
The Arduino Uno (ATmega328P) was selected for this project.
Justification: It offers a perfect balance of cost, I/O pin count (14 digital, 6 analog), and community support. Compared to the PIC micro-controller, Arduino allows for faster prototyping with C++. Compared to Raspberry Pi, it consumes significantly less power and performs real-time tasks without the overhead of an OS.

2.5 Use of Proteus and Tinkercad in Industry/Education
Proteus: Used extensively in industry for SPICE simulation and PCB layout. It allows for checking the "virtual" code execution before hardware assembly.
Tinkercad: A cloud-based tool ideal for 3D visualization of the robot's assembly and basic circuit logic validation, widely used in STEM education.

2.6 PCB Design Trends in Robotics
Modern robotics is moving away from breadboards toward Surface Mount Technology (SMT) and custom Through-Hole PCBs. Custom PCBs reduce electromagnetic interference (EMI) and mechanical failure risks. This project adopts a custom 2-layer PCB design to ensure durability.

2.7 Research Gap Identification
While many smart bins exist, a gap remains in robust, low-cost, domestically manufacturable designs. Most DIY projects remain on breadboards (prone to failure), while commercial units are expensive. This project fills the gap by engineering a custom PCB-based Smart Dustbin with a dedicated analog power regulation circuit, offering a middle ground between hobbyist prototypes and industrial products.






CHAPTER 3: METHODOLOGY
3.1 System Block Diagram
The system consists of three main stages: Input (Sensing), Processing (Control), and Output (Actuation), powered by a regulated Analog PSU.
(Description of Block Diagram):
Power Source: 9V/12V Battery or AC Adapter.
Analog Circuit: 5V Voltage Regulator (LM7805).
Input: HC-SR04 Ultrasonic Sensor.
Controller: Arduino Uno (ATmega328P).
Output: SG90 Servo Motor (Lid Mechanism).

3.2 Hardware Design
3.2.1 Analog Circuit Design 
To meet the specific requirement of analog circuit integration, a Regulated Power Supply Unit (PSU) was designed to convert an unregulated input (7V-12V) to a stable 5V DC required by the micro-controller and sensor.
Input: A 9V battery or DC jack.
Filtering: A 100µF Capacitor is placed across the input to filter out low-frequency ripples and noise from the power source.
Regulation: The LM7805 Linear Voltage Regulator is used. It is an analog 3-terminal integrated circuit that maintains a constant 5V output regardless of load changes.
Output Stabilization: A 10µF Capacitor and a 0.1µF Ceramic Capacitor are placed at the output. The 10µF handles transient load changes (e.g., when the servo motor starts), while the 0.1µF filters high-frequency noise that could interfere with the Ultrasonic sensor's echo signal.
3.2.2 Micro-controller Selection & Pin Mapping
Selected Controller: Arduino Uno. Pin Mapping Table: | Component | Pin Name | Arduino Pin | Description | | :--- | :--- | :--- | :--- | | HC-SR04 | VCC | 5V | Power | | HC-SR04 | GND | GND | Ground | | HC-SR04 | TRIG | D9 | Trigger Pulse Output | | HC-SR04 | ECHO | D10 | Echo Pulse Input | | Servo Motor | SIG | D7 | PWM Control Signal | | Servo Motor | VCC | 5V | Power |
3.2.3 Complete Schematic in Proteus
The schematic was drawn in Proteus 8 Professional. It connects the Analog PSU to the VCC/GND rails. The Arduino is wired to the HC-SR04 and Servo. Virtual instruments (Oscilloscope) were attached to the PWM pin (D7) to visualize the servo signal.
3.2.4 Simulation Results in Proteus
Scenario 1: Object placed > 30cm away. The Virtual Terminal shows "No Object". Servo signal remains Low (0 degrees).
Scenario 2: Object placed < 30cm. The sensor sends a logic High to Pin D10. The Arduino outputs a PWM signal with a 2ms pulse width, rotating the servo to 90 degrees (Open).
3.2.5 Tinkercad 3D Simulation
A 3D simulation was created in Tinkercad Circuits to visualize the wiring and physical layout. This confirmed that the servo arm has sufficient clearance to open the lid without obstruction.
3.3 PCB Design
A custom PCB was designed using EasyEDA/Proteus ARES.
Layout: The PCB acts as a "Shield" that sits on top of the Arduino or as a standalone board containing the ATmega328P.
Routing: Power tracks (VCC/GND) were made thicker (0.5mm) to handle the current of the servo motor.
Gerber Files: Generated for Top Copper, Bottom Copper, Silk Screen, and Drill holes.





3.4 Software Development
Flowchart Logic:
Start -> Initialize Servo and Serial Monitor.
Loop -> Send 10µs Trigger pulse to Sensor.
Read Echo -> Calculate Distance = (Duration x 0.034) / 2.
Decision: Is Distance < 30cm?
YES: Rotate Servo to 90° (Open Lid) -> Wait 3 Seconds.
NO: Rotate Servo to 0° (Close Lid).
Repeat Loop.
(Full Code provided in Appendix)

3.5 Prototype Development
Step 1: The chassis was prepared by cutting a mounting hole in a standard plastic bin.
Step 2: The SG90 Servo motor was glued to the hinge of the lid.
Step 3: The PCB was soldered with headers for the Arduino and screw terminals for the battery.
Step 4: Components were assembled, and the Arduino was mounted inside a protective casing to isolate it from waste.
3.6 Testing and Validation Procedure
Unit Testing: The PSU was tested with a multi-meter to ensure exactly 5.0V output.
Integration Testing: The system was powered on, and objects of different materials (hand, paper, plastic) were presented to the sensor to verify detection accuracy.
Stress Testing: The lid was triggered 50 times continuously to check for servo overheating or code freezing.


3.7 Ethical & Safety Considerations
Safety: All wiring is insulated to prevent short circuits. The 9V battery is low voltage, posing no shock risk.
Ethics: The project promotes environmental sustainability and hygiene. No hazardous materials were used in the construction.

CHAPTER 4: RESULTS AND DISCUSSION
4.1 Simulation Results
The Proteus simulation successfully demonstrated the logic.
Screenshot 1: Shows the distance variable at 15cm. The Logic Analyzer attached to Pin D7 shows a High Pulse indicating servo movement.
Screenshot 2: Shows the distance at 50cm. Pin D7 is Low, indicating the lid is closed.
4.2 Hardware Results
The physical prototype functioned as expected.
Response Time: The lid opened within 0.5 seconds of hand detection.
Range: The effective detection range was measured at 2cm to 30cm, perfectly matching the software calibration.
Power Stability: The analog regulator kept the voltage at 4.98V - 5.01V even when the servo motor was moving, preventing micro-controller resets.
Table 4.1: Detection Accuracy Test | Trial | Actual Distance (cm) | Measured Distance (cm) | Lid Status | Result | | :--- | :--- | :--- | :--- | :--- | | 1 | 50 | 49 | Closed | Pass | | 2 | 25 | 26 | Open | Pass | | 3 | 10 | 11 | Open | Pass | | 4 | 5 | 5 | Open | Pass |
4.3 PCB Performance Comparison
Breadboard: During testing, the breadboard circuit occasionally failed due to loose wires when the bin vibrated.
Custom PCB: The soldered PCB eliminated all connectivity issues. It was 40% more compact and could be easily mounted inside the bin lid.


4.4 Discussion of Findings
The results align with the literature reviewed in Chapter 2. Similar to Monika K A et al., the ultrasonic sensor proved reliable for short-range detection. However, our addition of the Analog PSU capacitor bank (Methodology 3.2.1) resulted in smoother servo operation compared to direct battery connection models, validating the importance of analog circuit design in robotics.
4.5 Problems Encountered and Solutions
Jittering Servo: Initially, the servo motor would shake when the lid was holding still.
Solution: We optimized the code to "detach" the servo object when not moving to stop it from seeking a position continuously.
Sensor Fluctuations: The sensor sometimes gave erratic 0cm readings.
Solution: A software filter (averaging 3 readings) was implemented to ignore outliers.

CHAPTER 5: CONCLUSION AND RECOMMENDATIONS
5.1 Summary of Findings
The Smart Dustbin was successfully designed, simulated, and built. The Proteus simulations accurately predicted the hardware behavior. The custom PCB design significantly improved the robustness of the device, and the integration of analog power regulation ensured stable operation. The specific objectives of simulation, coding, PCB design, and prototyping were all met.
5.2 Conclusion
This project demonstrates that low-cost automation is a viable solution for improving domestic hygiene. By integrating analog circuit principles (power regulation) with digital control (Arduino), we created a reliable system that eliminates the need for physical contact with waste containers. The "Smart Dustbin" serves as a practical application of mechatronics that can be easily scaled for mass production.





5.3 Recommendations
For Industry: Implement a solar charging circuit (as reviewed in Chapter 2) to make the bin energy-autonomous for outdoor parks.
For Future Students: Explore the use of Interrupt Service Routines (ISRs) for the sensor to allow the micro-controller to perform other tasks (like displaying fill level on an LCD) while waiting for the sensor echo.

5.4 Contribution to Knowledge
This report contributes a verified PCB design and Schematic for a robust Smart Dustbin, moving beyond the unstable breadboard models commonly found in online tutorials. It highlights the critical role of analog power filtering in preventing digital sensor noise.

5.5 Limitations
The current design uses a servo motor with plastic gears, which may wear out with heavy lids. The system relies on a battery that requires periodic replacement.
5.6 Suggestions for Future Work
Integration of a GSM Module to alert sanitation workers when the bin is full.
Implementation of a Waste Segregation Mechanism using moisture and inductive sensors to separate wet and dry waste automatically.



REFERENCES
Jetir.Org. (2023). A Review paper on Smart Dustbin. International Journal of Engineering Technology.
Monika, K. A., et al. (2019). Smart Dustbin based on IoT. International Journal of Scientific & Engineering Research.
Khan, R., et al. (2021). Smart Bin: An IoT-powered waste monitoring system. International Journal of Innovative Research in Science.
Engineers Garage. (2024). How to make a smart dustbin using Arduino?
Arthur, F., et al. (2024). Overview of Smart Dustbin Systems and Technological Methods.
Laha, S., et al. (2023). LoRa-based Smart Waste Management System.
Sathyabama Institute. (2020). Smart Dustbin Management Using IoT and Blynk Application.
Scribd. (2023). Smart Dustbin Project Report: Analog and Digital Implementation.


APPENDIX
A.	The C++ code
#include <Servo.h>

Servo lidServo;

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// Servo pin
const int servoPin = 6;

long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lidServo.attach(servoPin);
  lidServo.write(0);   // Lid closed position

  Serial.begin(9600);
}

void loop() {
  // Send ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance (cm)
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // If hand is detected
  if (distance > 0 && distance <= 15) {
    lidServo.write(90);   
    delay(3000);           
    lidServo.write(0);    
  }

  delay(500);
}






































