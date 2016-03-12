# Artificial-Potential-Field
<b>Implementation of Artificial Potential Field (Reactive Method of Motion Planing) </b>

For basics and working of Potential Field Motion Planning one can refer to http://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

This is basic implementation of potential field motion planning. Here we condsider our bot as positive charge, and goal as negatively charged body and all obstacles as positively charge bodies. 
This way goal will attract bot but obstacles will repel it from itself. Hence bot will reach to goal avoiding obstacles.

```
Attractive force = 
- tau(q(current) -q(goal)), if d(q(current), d(q(goal), q(current)) <= d*
- d*(tau*(q(current)-q(goal)))/d(q(current), q(goal)), if d(q(goal), q(current)) > d*

Repulsive Force =
- n(1/Q* - 1/D(q))*(1/D(q))^2* d'(q), if D(q) < Q*
- 0, if D(q) >= Q*
```

<b> Prequisites</b>
- Python
- OpenCV
- Numpy
- Matplotlib

<b> Input Images </b>
It will take all images in root folder as input images.

Sample Input Images:

![alt text][logo]
[logo]: 1.jpg "Sample Image"

![alt text][logo]
[logo]: 2.jpg "Sample Image"

![alt text][logo]
[logo]: 3.jpg "Sample Image"

Output Images:

![alt text][logo]
[logo]: output/1.jpg "Sample Image"

![alt text][logo]
[logo]: output/2.jpg "Sample Image"

![alt text][logo]
[logo]: output/3.jpg "Sample Image"

