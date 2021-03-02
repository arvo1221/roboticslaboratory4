# roboticslaboratory4
Robotics Laboratory 4 Term Project.
참가인원 : 16 남인수, 16 이동규, 16 최승완, 16 함병일

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
<img src="https://user-images.githubusercontent.com/54669783/109646380-78193f80-7b9b-11eb-925d-1b01e448e1ac.png" width="800" height="400" />  
    
## MFC UI
![MFC UI](https://user-images.githubusercontent.com/54669783/109646614-c29abc00-7b9b-11eb-8282-a2e746ce77b9.png)

## Project GIF (Simulation)

1. Forward Kinematics (Joint Angle : (180, 0) -> Position : (-2, 0, 0))
<img src="https://user-images.githubusercontent.com/54669783/109646259-515b0900-7b9b-11eb-95c3-a0e34038876a.gif" width="800" height="400" /> 

2. Inverse Kinematics (Position : (0, 2, 0) -> Joint Angle : (90, 0))
<img src="https://user-images.githubusercontent.com/54669783/109689727-f17b5700-7bc8-11eb-8135-dcb28aae99fc.gif" width="800" height="400" /> 

3. Target Position에 수렴 (Target Position : 0 degree -> 90 degree)
<img src="https://user-images.githubusercontent.com/54669783/109646195-39838500-7b9b-11eb-84f2-f2cd8945021f.gif" width="800" height="400" /> 

4. Torque Saturation (0.05 -> 0.01)
<img src="https://user-images.githubusercontent.com/54669783/109646219-42745680-7b9b-11eb-8f94-edfe41c72047.gif" width="800" height="400" /> 

5. 외란에도 Target Position에 수렴 (Target Position : (180, 0))
<img src="https://user-images.githubusercontent.com/54669783/109646251-4e601880-7b9b-11eb-9ec9-ec5086c3338b.gif" width="800" height="400" /> 


## Project GIF (HardWare)

1. Target Position Set
<img src="https://user-images.githubusercontent.com/54669783/109646235-486a3780-7b9b-11eb-80da-80311555f551.gif" width="400" height="800" /> 

2. 외란에도 Target Position에 수렴
<img src="https://user-images.githubusercontent.com/54669783/109646181-34bed100-7b9b-11eb-9826-0a52712673ee.gif" width="400" height="800" /> 

3. Torque Saturation
<img src="https://user-images.githubusercontent.com/54669783/109646200-3c7e7580-7b9b-11eb-85c6-b19f5b112b95.gif" width="400" height="800" /> 








