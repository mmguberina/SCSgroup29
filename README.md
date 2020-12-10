# SCSgroup29
The group project for the Simulation of Complex Systems course

Model
-----
write that in the report

Benchmarks
----------
NOTE: all must be tested in the same environment, but test them all on 
many different environemnts and then average results because that's the only fair thing

the benchmarks are everything the final model is build upon, i.e.
- just random swimming, all kinds
- random swimming plus artificial potential fields
- random swimming with new states like unstucking
- combinations of all of it

another cool benchmark would be an 'all knowing' version where
items are assigned and then picked up and retrieved with astar. 




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

