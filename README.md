# AIfrend clever lamp
Creating the best friend for user desck. We will use wx250s robotic arm from trossen robotics, with kodak projector min attached to the end-effector to create a smart lamp to help the user with his work.

## Resources

- Buy [Kodak Projector mini 75](https://www.amazon.es/Proyector-ultraport%C3%A1til-pulgadas-recargable-integrado/dp/B078NCG82N/ref=sr_1_1_sspa?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3S9FWZSE5RNKX&dib=eyJ2IjoiMSJ9.O8ucHWBeDVklyy4E2G41tBv8Ia-koCQlhFK2-Gaa-sfu27Qibdr6sjtje2vBBshUx0CzYE6RjgCFtxasghLWDDIU2orVaDq9UBfuy8Fgbt6y-cD9T7YY4B4xmdCXQINHssZoAu0IbHls-0fcyCdUTBtkNML2p2IcDDLb16Sl-fBh9IGZmtBXrpvIQOudGI3tpmXbGAN9PZPQL42tDny_-oY1NeIJ1Qwu9Nog4Lcj6VHFwG8UjK0ffjWMBQBGJ1QhjWz8JBp1HbcoFbzwWYb2BXxU-_6vTjoVKsnuKYXFLrc.HbABcLTElT1qpDQNPyOYGagCEX3jk-Wce0fHaSc58V8&dib_tag=se&keywords=kodak+projector&qid=1716604099&sprefix=kodak+projector%2Caps%2C132&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)
- download [kodak manual pdf](demos/kodak_manual.pdf)
- [natnet_ros_cpp](https://github.com/L2S-lab/natnet_ros_cpp)

## Intro

This project introduces the Clever Lamp, an innovative robotic lighting system designed to be the perfect companion for users. The system features a WX250s robotic arm from Trossen Robotics, equipped with a Kodak Mini projector mounted on its end-effector. The Clever Lamp combines advanced robotics with customizable projection capabilities, creating a versatile and interactive lighting solution.

The robotic arm's precise movements allow the projector to illuminate and transform the surrounding environment with tailored images and videos. Users can effortlessly manipulate the lamp's position and projection content, offering a unique and personalized lighting experience. The simplicity of the 3D design ensures ease of use and installation, while the customization options open up a wide range of applications, from mood lighting and entertainment to educational and professional uses.

By integrating cutting-edge robotics with user-centric design, the Clever Lamp offers immense potential for creative and practical applications, making it an indispensable addition to modern living spaces.

## Before running the code

To run the code you will need some prerequisites:

1. Optitrack system: It is a system of camares that detect position and orientation of cetain objects thank to capability of reflection of the ball markers.
2. Install [natnet_ros_cpp](https://github.com/L2S-lab/natnet_ros_cpp) ros package to send messages from Optitrack to your `roscore`.
3. For a easyer user interaction we 3D printed our [universal marker](https://github.com/IERoboticsAILab/3d_printing_designs/blob/main/files/optitrack/Universal_Marker_3.stl). It is an object that will be easily detected from Optitrack. We call it `umh_2`.
4. For a easyer user interaction we 3D printed our [custom wx250s base](https://github.com/IERoboticsAILab/3d_printing_designs/blob/main/files/WX-250_robot_garden/base/bottom_base_WX-250_for%20Optitrack.stl), for the robot arm. It has M3 scrues to host marker balls in place and detect position of robot base. We call it `real_base_wx250s`.

## Move robot

## Computer vision