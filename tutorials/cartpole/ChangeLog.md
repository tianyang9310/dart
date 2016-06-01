# Change and debug log

1. remove GLOBAL variable and use dynamic_pointer_cast to dynamic cast pointer

2. assure constants are initialized before used
	
	*. '''Qf.setIdentity()*5''' doesn't make any sense.

3. regard discreteized cost as continuous one, therefore for state cost it should have delta_t term

4. forcefully initiate x to set correct memory useage

5. prove dynamics and dynamics derivative

6. a major bug in backward pass, that is the last Vx and Vxx are not initiated correctly

7. __when using comma initialization, it is required that input all the data. It is not allowed to input one column at each for loop__

	*. Because finite difference method is a general funciton. Basically it handles all kinds of functions. So I use function template

	*. StepDynamics, StepCost, FinalCost have different arguments

	*. Lambda expression is used to encapsulate function and their arguments. Thereafter, pass into finite difference function only a function handle and an argument
	
8. Only first order derivative dynamics is considered

9. Second order finite difference has some problem, it is not accurate and doesn't behavior correctly

10. parallel computing alpha

11. state cost is very important. If I multiply it with a small constant such as delta_t it will converge much fast and stably
