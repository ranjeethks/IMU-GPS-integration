# IMU-GPS-integration
Loose-coupled integration of GPS and 3D RISS IMU solutions

This project explores the benefit of integrating two navigation solutions, INS and GPS, which have complementary characteristics. INS has better short term accuracy, with long term errors increase with time. On the other hand, the GPS has good long term accuracy. In this project, INS is integrated with GPS solution with a loosely coupled Kalman Filter (KF) approach. A 3D-RISS architecture is integrated in this approach is demonstrated to show the improvements in PVA accuracy, which is better than either INS or GPS standalone solution.

See a detailed report in [writeup report](https://github.com/ranjeethks/IMU-GPS-integration/blob/master/Project_Report_Ranjeeth.pdf)
