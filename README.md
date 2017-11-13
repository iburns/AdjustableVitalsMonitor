# Adjustable Vitals Monitor - Project AVM
This project was created during HealthHacks 2017, a 24 hour hackathon, as a solution to a challenge presented by OpenEMR: creating a device that measured vitals and 

Please check out the full write up here: https://devpost.com/software/project-avm-adjustable-vitals-monitor

Write up thanks to: Omar Karim

## Inspiration
Most members of our group have varying experience in the medical field, as well as the pre-hospital setting. The number one thing in medicine that's hammered home is the importance of vitals; you can't step into any room of a hospital or ambulance without seeing a giant screen displaying the patient's vitals. In the developed world, a standard set of vitals is part of the routine check-up. But what happens when you can't afford that trip to the doctor, or even worse, your doctor doesn't have the equipment to accurately measure and track vitals? Even worse, what happens when you're thrown abroad on a mission trip and are told to start treating patients who you can't even begin to understand?

## What it does
Enter: the Adjustable Vitals Monitor, or AVM. The AVM streamlines the process of vitals monitoring by measuring SpO2, Heart Rate, Blood Pressure, and Temperature at the click of a button. The device can hold and store data for three trial runs, and can either wirelessly transmit the data to a central server or separate device, such as a smartphone via bluetooth. Or, if the area is underdeveloped, the device can be modified to simply cycle through previous vital results, which can then be transcribed by hand.

In addition to the four basic vital signs, the AVM comes equipped with a Severity measurement. The Severity counter works by taking into account all four vital signs, computing their percent difference from the expected value of the patient's age group, and standardizes that patient's severity of injury on a scale of 0 to 1, with 0 indicating a perfectly healthy individual, and 1 meaning no signs of life detected. In addition, the AVM then displays these terms from greatest deviation from normal vitals to the least, which can tell the provider which vital sign deviation is contributing the greatest to the patient's illness. While the calculation is not perfect, the severity measurement allows EMTs and lay responders to better triage patients based on which of the three major vital signs is most contributing to the patient's distress. This is especially useful for children, who often have difficulty expressing accurately expressing the source of their pain when placed in a stressful or unfamiliar environment, such as an ambulance or hospital. Thus, the AVM can streamline the process of patient triage, while also cutting down the time between vitals calculation and delivery of treatment.

## How we built it
Various Sensors were added to an Arduino 101 board hooked to a base shield and breadboard. These sensors were placed inside a 3D printed, size-adjustable band for the extremities. Housed inside this band, the sensors measured the vital signs needed, such as Heart Rate, Temperature, Blood Pressure, and SpO2. The physical board was mounted on top of the wrist band. All of this can be powered with a 9V battery. The arduino would output the vital signs on a LCD display.

## Challenges we ran into
To start off the hackathon, the Arduino driver download server was down so we spent almost two hours looking for alternative sources to be able to start our project. From there, our next big challenge was trying to measure blood pressure, because that was a sensor that wasn't available to use. We needed to identify a way to model and report blood pressure. However, we were not able to build a cuff that could measure blood pressure and incorporate it into the band. Instead, with the help of the mentors, we were able to relate systolic blood pressure to the heart rate and run regressions to find conversion equations for each age group. Lastly, as we were finishing our project, hardware issues caused our LCD display to fail. We weren't able to cycle through vital signs as was possible earlier in the hackathon.

## Accomplishments that we're proud of
Our group is especially proud of the fact we were able to derive two of the four major vital signs even without the actual instruments that would be needed to directly retrieve the results. Though the calculations are not 100% accurate, they are accurate to the degree that an EMT or aid worker would need to properly deliver care. In addition, our Severity measure, while not a purely objective measure of pain, still serves to tell first responders and care workers what specific body system is being compromised, even in the presence of a language barrier. Two numbers and a letter/letters can now mean the difference between life or death for a child. Our group is also proud of overcoming obstacles in the 3D Printing process in order to finally produce a working prototype extremity band. Finally, our coder Ian deserves immense praise for overcoming all sorts of technical issues and bugs in order to finally calibrate the Arduino software to output the vital readings.

## What we learned
Throughout this project, we learned to how successfully code in Arduino despite never having prior experience. Additionally, we successfully produced our first 3D Printed model with CAD software despite roadblocks during the process. We also learned how to apply lessons from our classes such as Engineering 102 and Physics 207 in order to derive the necessary voltage to power certain components of the device, as well as how to derive an equation for severity.

## What's next for Project AVM (Adjustable Vitals Monitor)
Currently, AVM is only calibrated for one specific age group, as the equations we inputted vary based on age group. Our next step would be to program the AVM to accept an age input prior to activating, which would then allow the AVM to run the appropriate algorithm to determine the measured Blood Pressure and Temperature. Additionally, we hope to make variants of the band that can be applied to patients with obesity or COPD, as currently the AVM cannot give accurate measurements for such patients due to their poor blood perfusion and inherently low SpO2 values, respectively.
