# Homework2

## Question 1

### (a) Imagine you want your robot to perform navigation tasks, which approach would you choose?

#### 	It depends. If the robot is going to work in a static room, then it just need to follow the plan. If it’s going to work in a dynamic room, then behavior based paradigm must be used to avoid obstacles. And if working range of the robot is large, then we should carefully use behavior based paradigm since it often leads to local minimal. Optimization based planning should take over these tasks then.

### (b) What are the benefits of the behavior based paradigm?

#### 	a. Robustness : help robots survive and complete multiple tasks in complex dynamic environment.

#### 	b. Modularity : provide well softward design principles.

#### 	c. No memory needed : reduce the response time and enhance endurance.

### (c) Which approaches will win in the long run?

#### 	In the long run, hybrid reactive paradigm will win for sure. Hybrid reactive paradigm combines the advantages of optimization based paradigm and behavior based paradigm. World model will be used for planning while behavior based paradigm can reactive to multiple events and address them quickly.

## Question 2

### (a) How to generate uniform, perpendicular, attractive, repulse, tangential  forces for a robot and obstacles with known positions? Provide related  mathematical formulas.

$$
Uniform: \overrightarrow v=\overrightarrow v_0\\
Perpendicular:\overrightarrow v=\eta  \overrightarrow h \frac{v_p}{h^3}\\
Attractive:\overrightarrow v = \xi  (\overrightarrow r - \overrightarrow r_0)|\overrightarrow r - \overrightarrow r_0|*v_0\\
Repulsive:\overrightarrow v = \eta  (\overrightarrow r - \overrightarrow r_0)\frac{v_0} {|\overrightarrow r - \overrightarrow r_0|^3}\\
Tangential:\overrightarrow v=\overrightarrow w_0\times(\overrightarrow r - \overrightarrow r_0)\\
$$

