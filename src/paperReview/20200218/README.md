# Prediction of RTK-GNSS Performance in Urban Environments Using a 3D model and Continuous LoS Method ([pdf](20191225_ionitm_e_v9_2.pdf))

## Abstract
To utilize RTK-GNSS in urban areas, it is important to predict areas in which it can be used. The performance of RTK- GNSS depends on the geometry and number of visible satellites and signal quality. These parameters can potentially be predicted using simulations that consider the relative geometry between the receiver and surrounding objects.
In this study, we first verified whether the GNSS signal quality can be correctly predicted using 3D models of buildings and measurement data. Subsequently, we verified whether the FIX status of RTK can be correctly predicted. The results show that the number of the measured and predicted satellites that have good signal quality was in agreement at least 87.8% of the time. We assessed and categorized the RTK-GNSS fixing status using the number of usable satellites. A comparison of the RTK fixed status estimation, using the actual measurements and those from the simulation, agreed within 83.9% of the total. 

**Authors**
Rei Furukawa is a GNSS simulation software developer at KOZO KEIKAKU ENGINEERING Inc. He graduated from Seikei university with B.S. and M.S. degrees in Mechanical Engineering. From October 2016, he is also taking his doctorate course on GNSS at the Tokyo University of Marine Science and Technology.
Nobuaki Kubo received his Doctorate in Engineering from the University of Tokyo in 2005. He resided at Stanford University in 2008 as a visiting scholar. He is now a professor at the Tokyo University of Marine Science and Technology (TUMSAT), specializing in GPS/GNSS systems. His current interests are high accuracy automobile navigation using RTK and multipath mitigation techniques. Ahmed El-Mowafy is an Associate Professor at Curtin University.His main areas of research are Integrity monitoring and quality control of Global Navigation Satellite Systems (GNSS- GPS) apllications, precise positioning and navigation, attitude determination, integration of GPS with other sensors (e.g. IMU), machine automation, deformation analysis, and hydrographic surveying.

<p align="center">
  <img width="712pix" src="framework1.png">
</p>

<p align="center">
  <img width="712pix" src="result1.png">
</p>

## comments
This is the work from Prof. Kubo about employing the 3D building models to help the GNSS RTK by detecting the LOS satellites. This is only a preliminary result.

**Advantage**: fix rate is improved accordingly.
**Question**: 
- Q1: use the 3D building model in a more intelligent way? employ the 3D building model during the LAMBDA algorithm