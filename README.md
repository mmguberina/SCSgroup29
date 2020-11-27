# SCSgroup29
The group project for the Simulation of Complex Systems course

First version
-------------
- discrete
- let the robot controler be finite state automata 
  with states: Searching, Delivering.
-  First the search will be a random walk and deliver we'll be moving in 1-move increments
  to the desired location.
- The robots are dimeonsionless.
- For physical realism, there are walls at the end of the grid.

Benchmarks
----------
- the first version can also serve as a benchmark just to prove our algorithms are better than random.
- the other benchmark would then be the case where everything is known
so that robots do not explore, but are assigned to get to the item closest to them
----> what happens if 2 robots are closest to the same item? how is that decided?
- in general, how to measure success? 
the speed per item is ok if we are comparing different algorithms in the same situation (same
grid size, expected number of items etc), but how to compare
the same algorithm on various situations?

- how to include package placement in the this? 
if we have developed the robot algorithms then we can ask how to place the items
so that the delivery time is the lowest



in the following assume every item can be picked up by 1 robot
-------------------------------------------------------------
PSO inspired continuous model
--------------------------

====> try the torque model from SCS homework3 first!!!
====> active-active torque from hw for attraction to items,
	  active-passive for robots

- robots are circles (easy to check distances)
- no robots (circles) can overlap
- robots move in a directed brownian motion, but so that
they act like charged particles, meaning:
	- robots repel each other by some function of distance
	- items atract robots by some function of distance
- the atracting and repeling happens is not calculated globally, but
by each individual robot assuming that a robot has some field of vision
and is only influenced by object in it (the field of vision can be a circle)
- the atraction/repulsion function have some number of parameters and
those can be handpicked, evolved or learned



PSO inspired discrete model
--------------------------
- robots are points on verteces 
- no robots can be on the same vertex
- robots move in as a random walk on the lattice, but so that
they act like charged particles, meaning:
	- robots repel each other by some function of distance
	- items atract robots by some function of distance
- the atracting and repeling happens is not calculated globally, but
by each individual robot assuming that a robot has some field of vision
and is only influenced by object in it (the field of vision can be a circle
in manhatan (L1) norm, i.e. von neumann neighbours up to some degree)
- the motion is still stochastic, but the probabilities of moving in a
certain direction is influenced by visible robots and items
- the atraction/repulsion function have some number of parameters and
those can be handpicked, evolved or learned


Simple all-knowing continous/discrete model
------------------------------------
- all item positions are known to every robot, meaning
a central algorithm assigns robots items they need to pick up
- an algorithmically hard problem --- what to do when the same item
is the closest one to more robots? 
---> possible solution: make a list of closest items to some robot
and then if some other robot is closer to the first item on the
list, take the second one and so on
---> alternative: make a list of closest robots to an item and then
make the choise
- what are the complexities of these algorithms?
- are the guaranteed to pick up all items?
- is there some guarantee of optimality?




Adding complexity, easier to harder
-----------------------------------
1. making robots dimensional in both cont and discrete cases
2. prioritizing items, in general and/or with delay penalties
3. adding bariers, ex. a grid of shelves on which the items are
4. having items which more robots need to carry


Programing remarks
------------------
Paralelism
---------
- in order to do all this on time, parallelism is necessary. 
fortunately the need for this comes from testing, which means we just 
need to run more simulations at once and average the results,
meaning that the processes do not need to communicate. in other
words, it is really easy to implement paralelism,
one just needs to know the API.

Data structures
-----------------
- object oriented approach is the most straightforward, but
does not exist in matlab (afaik), also could be slow
- sparse matrix reprezentation for locations of things on the grid?
