-module(pso).
-export([benchmark_strong/0, benchmark_weak/0, start/4, rastrigin/1, sphere/1, rosenbrok/1, styblinski/1]).

sphere(Coords) ->
    lists:sum([X*X || X <- Coords]).
    
rastrigin(Coords) ->
    A = 10,
    N = length(Coords),
    A*N + lists:sum([X*X  - A*math:cos(2*math:pi()*X)
	       || X <- Coords]).

rosenbrok(Coords) ->
    {_, Temp} = lists:split(1, Coords),
    {Coords1, _} = lists:split(length(Coords) - 1, Coords),
    Coords2 = Temp,
    CoordinateTuple = lists:zip(Coords1, Coords2),
    lists:sum([(100*(X_plusOne - X*X) + (X - 1)*(X - 1))
	       || {X, X_plusOne}  <- CoordinateTuple]).

styblinski(Coords) ->
    lists:sum([math:pow(X,4) - 16*math:pow(X,2) + 5*X || X <- Coords])/2.

benchmark_strong() ->
    IterationsStart = 1000,
    ActorsMax = 64,
    BenchmarkCount = 6,
    Particles = 64,
    [runBenchmark(IterationsStart,Actors , Particles, BenchmarkCount) || Actors <- lists:seq(1, ActorsMax)],
    ok.

benchmark_weak() ->
    IterationsStart = 1000,
    ActorsMax = 64,
    BenchmarkCount = 4,
    Particles = 64,
    [runBenchmark(IterationsStart,Actors , Particles*Actors, BenchmarkCount) || Actors <- lists:seq(1, ActorsMax)],
    ok.

runBenchmark(Iterations, Actors, Particles, IterationsPerSetup) ->
    Start = erlang:system_time(1000),
    Values = [start(Iterations, Actors, Particles, 1, benchmark) || _ <- lists:seq(1, IterationsPerSetup)],
    End = erlang:system_time(1000),
    io:format("Avg run time: ~ws, Actors: ~w, Iterations: ~w, Particles: ~w~n", [((End - Start)/IterationsPerSetup)/1000, Actors, Iterations, Particles]),
    Values.

avg(X) ->
    lists:sum(X)/length(X).

displayResults(Results, NumIterations, Actors, Dimensions, Iterations, ParticlesCount) ->
    Coords = [X || {X,_,_} <- Results],
    Values = [X || {_,X,_} <- Results],
    RunTimes = [X || {_,_,X} <- Results],
    Zipped = lists:zip(Values, Coords),
    ZippedSorted = lists:keysort(1, Zipped),
    {MinValue, BestCoords} = lists:nth(1, ZippedSorted),
    io:format("~n~n--- Execution results ---~n~nAverage optimum value found: ~w~nBest value found: ~w, ~n~nBest coordinates: ~w~n", [avg(Values), MinValue, BestCoords]),
    io:format("~nNumber of runs: ~w~nIterations per run: ~w~nNumber of Particles: ~w~n", [NumIterations, Iterations,ParticlesCount]),
    io:format("~nNumber of Actors: ~w~nNumber of Dimensions: ~w~n~nAvg run time / Actors: ~w~nTotal run time: ~ws~nAverage run time: ~wms~nExecution time per particle: ~w~n", [Actors, Dimensions, avg(RunTimes)/Actors, lists:sum(RunTimes)/1000, avg(RunTimes), avg(RunTimes)/(ParticlesCount)]).

start(IterationsPerRun, Actors, ParticlesCount, NumIterations) ->
    start(IterationsPerRun, Actors, ParticlesCount, NumIterations, normal).
start(IterationsPerRun, Actors, ParticlesCount, NumIterations, Type) ->
    Dimensions = 6,
    F = fun pso:rastrigin/1,

    % PSO constants:
    Omega = 1,
    PhiLocal = 1.9,
    PhiGlobal = 1.2,
    VelocityLimit = 1,
    Inertia = {0.9, 0.2},

    {InertiaStart, InetriaEnd} = Inertia,
    InertiaSub = (InertiaStart - InetriaEnd)/IterationsPerRun,  

    Results = [
	       main(Actors, Dimensions, F, {Omega, PhiLocal, PhiGlobal, InertiaStart, InertiaSub, VelocityLimit}, IterationsPerRun, ParticlesCount) 
	       || _ <- lists:seq(1, NumIterations)],
    [X || {_,X,_} <- Results],
    case Type of
	normal ->
	    displayResults(Results, NumIterations, Actors, Dimensions, IterationsPerRun, ParticlesCount);
	benchmark ->
	    ok
    end.

main(Actors, Dimensions, F, Constants, Iterations, ParticleCount) ->
    Start = erlang:system_time(1000),
    Pid = self(),
    NewActorCount = spawnWorkers(Pid, Dimensions, F, Constants, Iterations, ParticleCount, Actors),
    masterInit(NewActorCount, NewActorCount, {[], infinity}, Start).

spawnWorkers(Pid, Dimensions, F, Constants, Iterations, ParticleCount, Actors) ->
    Rems = ParticleCount rem Actors,

    Count1 = [spawn_link(fun() ->
		workerInit(Pid, Dimensions,F, Constants, Iterations, trunc(ParticleCount/Actors) + 1)
		end)
     || _ <- lists:seq(1, Rems)],

    case Actors =< ParticleCount of
	true ->

	   Count2 = [spawn_link(fun() ->
				workerInit(Pid, Dimensions,F, Constants, Iterations, trunc(ParticleCount/Actors))
			end)
	     || _ <- lists:seq(1, Actors - Rems)];
	    false ->
	    Count2 = [],
		     ok
    end,
    length(Count1 ++ Count2).

masterInit(0, WorkerCount, BestPos, Start) ->
    receiveLoop(WorkerCount, WorkerCount, BestPos, Start);

masterInit(RemActors, WorkerCount, {SwarmBestCoords, SwarmBest}, Start) ->
    receive
	{personalbest, {Coords, Value}} ->
	    case Value < SwarmBest of
		true ->
		    NewBest = {Coords, Value};
		false ->
		    NewBest = {SwarmBestCoords, SwarmBest}
	    end
    end,
    masterInit(RemActors - 1, WorkerCount, NewBest, Start).

receiveLoop(0, _, {Coords, Value}, Start) ->
    ElapsedTime = erlang:system_time(1000) - Start,
    {Coords, Value, ElapsedTime};

receiveLoop(RemActors, WorkerCount, BestPos, Start) ->
    {_, BestValue} = BestPos,
    receive
	done ->
	    receiveLoop(RemActors - 1, WorkerCount, BestPos, Start);
	{getSwarmBest, Pid} ->
	    Pid ! {swarmBest, BestPos},
	    receiveLoop(RemActors, WorkerCount, BestPos, Start);
	{personalbest, Coords, NewPBestValue} ->
	    case BestValue > NewPBestValue of
		true ->
		    receiveLoop(RemActors, WorkerCount, {Coords, NewPBestValue}, Start);
		false ->
		    receiveLoop(RemActors, WorkerCount, BestPos, Start)
	    end;
	E ->
	    io:format("Unknown error in receiveLoop, received: ~w~n", [E])
    end.

workerInit(Pid, Dimensions, F, Constants, Iterations, ParticleCount) ->
    {ParticleInfo, BestValue, BestCoords} = initParticle(ParticleCount, Dimensions, F, [], inf, []),
    Pid ! {personalbest, {BestCoords, BestValue}},
    workerLoop(Pid, ParticleInfo, F, Constants, Iterations, [], getSwarmBest(Pid)).
    
initParticle(0, _, _, ParticleInfo, BestValue, BestCoords) ->
    {ParticleInfo, BestValue, BestCoords};
    
initParticle(ParticleCount, Dimensions, F, ParticleInfo, BestValue, BestCoords) ->
    rand:seed(exs64),
    Coords = [10*(rand:uniform() - 0.5) || _ <- lists:seq(1, Dimensions)],
    Vels = [10*(rand:uniform() - 0.5) || _ <- lists:seq(1, Dimensions)],
    Value = F(Coords),
    PBest = {Coords, Value},
    case Value < BestValue of
	true ->
	    initParticle(ParticleCount - 1, Dimensions, F, [{Coords, Vels, PBest}] ++ ParticleInfo , Value, Coords);
	false ->
	    initParticle(ParticleCount - 1, Dimensions, F, [{Coords, Vels, PBest}] ++ ParticleInfo, BestValue, BestCoords)
    end.

getSwarmBest(Pid) ->
    Pid ! {getSwarmBest, self()},
    receive
	{swarmBest, {SwarmBestCoords, SwarmBestValue}} ->
	    ok;
	E ->
	    io:format("Unknown error in getSwarmBest, received: ~w~n", [E]),
	    SwarmBestCoords = [],
	    SwarmBestValue = inf
    end,
    {SwarmBestCoords, SwarmBestValue}.

workerLoop(Pid, [], F, {Omega, PhiLocal, PhiGlobal, Inertia, InertiaSub , VelocityLimit}, Iterations, UpdatedParticles, _) ->
    workerLoop(Pid, UpdatedParticles, F, {Omega, PhiLocal, PhiGlobal, Inertia - InertiaSub, InertiaSub , VelocityLimit}, Iterations - 1, [], getSwarmBest(Pid));
workerLoop(Pid, _, _, _, 0,_,_) ->
    Pid ! done;
workerLoop(Pid, [{Coords, Vels, PBest}|ParticleInfo], F, Constants, Iterations, UpdatedParticles, SwarmBest) ->
    {SwarmBestCoords, SwarmBestValue} = SwarmBest,
    NewParticle = {NewCoords, _, NewPBest} = updateFunction(Coords, Vels, Constants, SwarmBestCoords, PBest, F),
    {NewCoords, NewPBestValue} = NewPBest,
    case SwarmBestValue > NewPBestValue of
	true ->
	    Pid ! {personalbest, NewCoords, NewPBestValue};
	false ->
	    ok
    end,
    workerLoop(Pid, ParticleInfo, F, Constants, Iterations, [NewParticle] ++ UpdatedParticles, getSwarmBest(Pid)).

updateFunction(Coords, Vels, Constants, SwarmBestCoords, PBest, F) ->
    {PBestCoords, PBestValue} = PBest,
    {NewCoords, NewVels} = coordAndVelocityUpdate(Coords, Vels, PBestCoords, SwarmBestCoords, Constants), 
    NewValue = F(NewCoords),
    
    case PBestValue > NewValue of
	true ->
	    NewPBestValue = NewValue;
	false ->
	    NewPBestValue = PBest
    end,
    {NewCoords, NewVels, {NewCoords, NewPBestValue}}.
    
coordAndVelocityUpdate(Coords, Vels, PBest, SwarmBest, Constants) ->
    coordAndVelocityUpdate_(Coords, Vels, PBest, SwarmBest, [], [], Constants).

coordAndVelocityUpdate_([], _, _, _, NewCoords, NewVels, _) ->
    {NewCoords, NewVels};

coordAndVelocityUpdate_([C|Coords], [V|Vels], [P|PBest], [S|SwarmBest], NewCoords, NewVels, {Omega, PhiLocal, PhiGlobal, Inertia, InertiaSub , VelocityLimit}) ->
    RandPersonal = rand:uniform(),
    RandSwarm = rand:uniform(),
    NewVel = 
	min(VelocityLimit,
	Inertia*(
	Omega*V +
	PhiLocal*RandPersonal*(P - C) + 
	PhiGlobal*RandSwarm*(S - C))),

    NewCoord = C + NewVel,
    coordAndVelocityUpdate_(Coords, Vels, PBest, SwarmBest, [NewCoord] ++ NewCoords, [NewVel] ++ NewVels, {Omega, PhiLocal, PhiGlobal, Inertia, InertiaSub, VelocityLimit}).
