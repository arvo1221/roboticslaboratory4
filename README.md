# roboticslaboratory4
Robotics Laboratory 4 Term Project.

## Project Goal  
1. MFC를 통해 Atmega128과 Serial 통신하며, UI를 통해 Motor의 Torque, Velocity, Position Control을 한다.

2. Joint Angle을 입력했을 시 Forward Kinematics를 풀어 End Effector의 Position 출력하고, End Effector의 Position을 입력했을 시 Inverse Kinematics를 풀어 Joint Angle출력한다.

## Main Feature  
1. Atmega128과 MFC의 Serial 통신
2. Atmega128 내의 Cascade 방식으로 Current(PI Control), Velocity(PI Control), Position(PD Control) 제어
3. 2-DOF ARM의 End-Effector Position과 Joint Angle 간의 Forward Kinematics와 Inverse Kinematics 연산

  
## Using Language  
1. MFC : C++
2. Atmega128 : C
  
## System Architecture
<img src="https://user-images.githubusercontent.com/54669783/109646380-78193f80-7b9b-11eb-925d-1b01e448e1ac.png" width="800" height="500" />  
    
## MFC UI
<img src="https://user-images.githubusercontent.com/54669783/109646614-c29abc00-7b9b-11eb-8282-a2e746ce77b9.png" width="800" height="500" />  

## Project GIF  










