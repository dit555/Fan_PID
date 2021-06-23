# Fan_PID
Demo Link: https://youtu.be/Izt8bb94mMw  

## Floating a Ball Using a Fan:
In this project the goal is to float a ball by controlling a fan. This fan is controlled via two methods: a manual speed control via two buttons and a controller using PID.  
My simulation of the ball works by considering the acceleration of gravity on the ball and the acceleration imparted by the fan on the ball. The velocity of the ball and the resulting position are then calculated. In addition, the height of the ball can be displayed via setting the output of B equal to the height or translating the height into one out of eight positions on a LED bar.  
When setting A0 to 1 we enable manual control. In manual control, the default acceleration of the ball is 10m/s2 upwards to counteract gravity. Toggling A6 will cause the acceleration upward to increment. Toggling A7 will cause the acceleration caused by the fan to decrement. Toggling both or neither will result in the acceleration of the fan to return to the default acceleration. In one attempt of reaching the goal of 100 height with the ball I managed to get the ball to settle at 101 in 37 seconds:
 
<img src="https://user-images.githubusercontent.com/56750709/123151597-7a3c2e00-d418-11eb-9fc9-d119a64c683d.png" width=500>  

It was difficult to get the ball to stay on the goal as seen by both the overshoot and the undershoot. In addition I wasn’t able to stabilize it for a long time.  
When using PID to control the ball the simulation managed to stabilize at 100 in 9 seconds without overshoot using kp = 0.21, ki = 0.001, and kd =  2.5:  

<img src="https://user-images.githubusercontent.com/56750709/123151722-9fc93780-d418-11eb-8117-eb357e6ff035.jpg" width=500>  

However, it is important to note that due to software limitations only integer data types were used in these calculations. As a result there is considerable rounding error. In addition, to stabilize the ball at 100 I had to manually change the value of acceleration on the ball from the fan to make velocity zero and stabilize the ball at 100. Had it been possible to use float point data types it would have been possible to keep the ball stabilized at the 7 second mark.  
Overall, it is clear to see that using PID is superior to manually changing the acceleration on the ball from the fan.
