# Ch10_Introduction to Lie theory for Robotics

## Computer vision:

For the computer vision part of the simulation, we will be initializing 100 rotation matrices and taking their karcher mean on the manifold and then converging the camera pose to the resulting rotation matrix.

Scene: target.ttt

Code: target_api.py

The idea here is that we can maximize the number of points we sample by taking the average of multiple rotation matrices and then use or camera to look towards that pose. This is visually shown in the simulation snapshots below.

![1](https://user-images.githubusercontent.com/71635591/236719879-8d77e1ab-f0c9-44aa-9998-137b13b1c307.PNG)

Averaging rotation matrices and applying the control law to converge to the desired pose, we get

![2](https://user-images.githubusercontent.com/71635591/236719885-6752898a-f739-4a26-9216-f9d58f041101.PNG)



## Rigid body control:


We use the Dobot magician robotic arm for our simulation to show rigid body tracking. The desired pose to converge to is shown as the thin blue perspective sensor. This pose may be adjusted as desired.

Scene: rigid.ttt

Code: rigid_api.py

Snapshots showing convergence are shown below

![11](https://user-images.githubusercontent.com/71635591/236720782-2091f724-7981-4e0a-8fb8-d8b2ff5366fe.PNG)

After deploying the control law derived on SO(3), we get


![22](https://user-images.githubusercontent.com/71635591/236720865-16eb6a40-c958-4666-a005-1847d569ca61.PNG)





