# Intelligent-Vehicle-Control

Intelligent vehicle algorithm design to achieve the following two functions:
- self-following
- self-parking 

## Platform 
- Visual Studio 2010
- CyberTORCS  

## Aim
- **following**: The average distance between the two cars is kept at 20m.
- **parking**: The vehicle is able to be parked into the parking space by tail or head. with high parking speed and accuracy (deviation of the four vertices from the standard position of the vehicle). 


## Algorithm 
### Part 1: self-following
- **speed control**  
The **relative speed**, **relative acceleration** and **relative distance on x axis** of the two vehicles are used as control variables for speed control. Adopted a **linear** combination of these three.   

```
acc = k_v1 * v2 + k_a1 * a1 + k_y1 * yError + k_i * yErrorSum + k_d * yErrorDiff; 
brake = (-1) * (k_v2 * v2 + k_a2 * a1 + k_y2 * yError + k_i * yErrorSum 
+ k_d * yErrorDiff);
```  

- **direction control**  
The **relative distance on y axis** of the two cars is used as direction control variable. The control uses a **linear** expression.
```
*cmdSteer = -kp_d1 * xError - ki_d1 * xErrorSum -extraControl_x; 
```
- **optimiaztion** 
1.  **Point brake** for sharp turning protection. 
2.  **Optimal gear shifting** to make the acceleration faster; when the distance is too far, drop it directly, making the slowdown faster. 

### Part 2: self-parking

- **speed control**  
Control the deviation between the **actual speed** and **expected speed**. 

- **direction control**  
Control the **yaw error between the parking space orientation and vehicle orientation** and the **y distance error between the vehicle center and parking space center***.

## Results
For the following part, the testing was done on 33 different leading cars. For the parking part, testing was done on 5 different parking places. There was a hidden test case on the final competition for both parts.

|           | following    | parking  |
| --------  | -----:          |-----:    |
| testing   | 1.26m Average error     | 10.032 |  
| final competition | 1.166m Average error  | 10.519 |
| ranking        | 3/33      |   6/33     |

*ps: score for parking is calculated by **parking time \*(1+10\* deviation distance / car length)**.*






