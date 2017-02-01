# Concurrent particle swarm optimization

This project contains an erlang program that runs particle swarm optimization concurrently.

In order to run the program, stand in the correct directory and type
erl in a terminal. After that, compile the program with c(pso).  Once this is done, the function pso:start/4 can  be  run.   The  four  arguments  this  function  accepts are:

• IterationsPerRun - number of iterations each particle should move beforeterminating.

• Actors - the number of erlang processes.

• ParticlesCount - number of particles.

• NumIterations -  number  of  times  the  algorithm  should  be  restarted. Restarting can be useful since it is possible that one run will have better initilalizations than the other
