package aco;

import java.util.ArrayList;
import java.util.Collections;

import aco.Ants.Ant;

import java.lang.management.*;



/**
 * ACO algorithms for the TSP
 * 
 * This code is based on the ACOTSP project of Thomas Stuetzle.
 * It was initially ported from C to Java by Adrian Wilke.
 * 
 * Project website: http://adibaba.github.io/ACOTSPJava/
 * Source code: https://github.com/adibaba/ACOTSPJava/
 */

/* this Eclipse project represents the ACS algorithm adapted from the mTSP_ACO MinMax global_new heuristic Eclipse project,
 * for solving the vehicle routing problem with time windows (VRPTW), but in which the number
 * of used vehicles (or salesmen) is fixed and is known a priori; this value is taken from the best
 * known solution reported in the literature for that particular VRPTW test instance
 * (see http://people.idsia.ch/~luca/macs-vrptw/solutions/welcome.htm and http://web.cba.neu.edu/~msolomon/heuristi.htm)
 * 
 * the quantity of the deposited pheromone and the best ant is taken according to the MinSum criterion
 * (the total sum of the length/cost of each travelled route/tour)
 * according to the mTSP_ACO MinMax global_new heuristic algorithm, the salesman and the city to 
 * be visited and added in its tour are selected simultaneously during the same step from the 
 * construction phase of solution, reflecting a jointly decision making
 */

/* contains the main entry point for the implementation of ACO for solving the VRPTW problem on instances
 * belonging to Solomon benchmark
 */
public class VRPTW_ACS implements Runnable {
	 /*
     * ################################################
     * ########## ACO algorithms for the TSP ##########
     * ################################################
     * 
     * Version: 1.0
     * File: main.c
     * Author: Thomas Stuetzle
     * Purpose: main routines and control for the ACO algorithms
     * Check: README and gpl.txt
     * Copyright (C) 2002 Thomas Stuetzle
     */

    /***************************************************************************
     * Program's name: acotsp
     * 
     * Ant Colony Optimization algorithms (AS, ACS, EAS, RAS, MMAS, BWAS) for the
     * symmetric TSP
     * 
     * Copyright (C) 2004 Thomas Stuetzle
     * 
     * This program is free software; you can redistribute it and/or modify
     * it under the terms of the GNU General Public License as published by
     * the Free Software Foundation; either version 2 of the License, or
     * (at your option) any later version.
     * 
     * This program is distributed in the hope that it will be useful,
     * but WITHOUT ANY WARRANTY; without even the implied warranty of
     * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
     * GNU General Public License for more details.
     * 
     * You should have received a copy of the GNU General Public License
     * along with this program; if not, write to the Free Software
     * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
     * 
     * email: stuetzle no@spam informatik.tu-darmstadt.de
     * mail address: Universitaet Darmstadt
     * Fachbereich Informatik
     * Hochschulstr. 10
     * D-64283 Darmstadt
     * Germany
     ***************************************************************************/
	
	private volatile boolean isRunning = true;
    
    /** indicates whether local search is used in the ACO algorithm **/
    public static boolean ls_flag = true; 
    
    //indicates whether the thread was restarted from outside
    private boolean threadRestarted;
    
    private VRPTW vrpInstance;
    

    public VRPTW_ACS() {}
    
    public VRPTW_ACS(boolean threadStopped, VRPTW vrpInstance_) {
    	this.threadRestarted = threadStopped;
    	this.vrpInstance = vrpInstance_;
    }
    
	//checks whether termination condition is met
    static boolean termination_condition() {   
    	//return (((InOut.n_tours >= InOut.max_tours) && (Timer.elapsed_time() >= InOut.max_time)) || (Ants.best_so_far_ant.tour_length <= InOut.optimal));
    	//return ((InOut.n_tours >= InOut.max_tours) && (Timer.elapsed_time() >= InOut.max_time));
    	//return (InOut.iteration >= InOut.max_iterations);
    	return Timer.elapsed_time() >= InOut.max_time;
    }
    
    static boolean isFeasible(VRPTW vrp, Ant a, int city, double beginService, double beginServiceDepot, int indexSalesman) {
    	boolean ok = false;
    	double currentQuantity;
    	ArrayList<Request> reqList = vrp.getRequests();
    	
    	//check upper bound of the time window of the next customer to be visited && 
    	//current capacity of the car && vehicle arrival time at the depot (the maximum total route time -> upper bound of the time window of the depot)
    	currentQuantity = a.currentQuantity.get(indexSalesman) + reqList.get(city + 1).getDemand();
    	if (beginService <= reqList.get(city + 1).getEndWindow() && currentQuantity <= vrp.getCapacity()
    			&& beginServiceDepot <= reqList.get(0).getEndWindow()) {
    		ok = true;
    	}
    	if (ok == false) {
    		if (beginService > reqList.get(city + 1).getEndWindow()) {
    			//System.out.println("City " + city + ": End time window constraint violated");
    		}
    		if (currentQuantity > vrp.getCapacity()) {
    			//System.out.println("City " + city + ": Vehicle capacity constraint violated");
    		}
    		if (beginServiceDepot > reqList.get(0).getEndWindow()) {
    			//System.out.println("City " + city + ": Maximum total route time constraint violated");
    		}
    	}
    	
    	return ok;
    }
    
    //generate the 3 weights to be used in the nearest-neighbour heuristic, when computing an initial solution
    //to the VRPTW problem
    static void generateInitialWeights(){
    	double nr1, nr2;
    	
    	nr1 = Math.random();
		while (nr1 == 0.0) {
			nr1 = Math.random();
		}
		Ants.initialWeight1 = nr1;          //weight for the distance between two nodes in the graph measure
		
        //nr2 should be in the range of (0, 1 - nr1)
		nr2 = Math.random() * (1 - nr1);
		Ants.initialWeight2 = nr2;              //weight for the time difference measure
		Ants.initialWeight3 = 1 - nr1 - nr2;   //weight for the delivery urgency measure
		//System.out.println("Initial weights are: " + Ants.initialWeight1 + " " + Ants.initialWeight2 + " " + Ants.initialWeight3);
    }
    
    //check if there is still an ant with left cities to visit
    static boolean isDone() {
    	boolean done = true;
    	
    	for (int k = 0; k < Ants.n_ants; k++) {
    		if (Ants.ants[k].toVisit > 0) {
    			return false;
    		}
    	}
    	
    	return done;
    }
    
    //add to the ant's solution the committed nodes from each tour of the best so far solution
    static void addCommitedNodes(Ant a, VRPTW instance) {
    	int index, city, current_city;
    	double distance, arrivalTime, beginService;
	    ArrayList<Request> reqList = instance.getRequests();
	    int startIndex = 1, pos;
    	
	    ArrayList<Integer> lastCommitedIndexes = new ArrayList<Integer>();
		for (int indexTour = 0; indexTour < Ants.best_so_far_ant.usedVehicles; indexTour++) {
			pos = Controller.getLastCommitedPos(indexTour);
			lastCommitedIndexes.add(pos);
		}
	    
    	for (int i = 0; i < lastCommitedIndexes.size(); i++) {
    		//we have at least one committed node in the i-th tour (i.e. tour with index i) 
    		index = lastCommitedIndexes.get(i);
    		if (index > 0) {
    			//if the number of vehicles (tours) from the ant solution is less than the index of the
    			//tour from the best so far solution, add new (empty) tours in the ant's solution
    			if (a.usedVehicles < (i + 1)) {
    				a.usedVehicles = i + 1;
    				for (int l = startIndex; l < a.usedVehicles; l++) { 
	    				a.tours.add(l, new ArrayList<Integer>());
	    				a.tours.get(l).add(-1); 
	    		    	a.tour_lengths.add(l, 0.0);
	    		    	a.currentQuantity.add(l, 0.0);
	    				a.currentTime.add(l, 0.0);
    				}
    				startIndex = i + 1;
    			}

    			int lastPos = a.tours.get(i).size() - 1;
    			current_city = a.tours.get(i).get(lastPos);
    			current_city++;
    			//add in the ant's i-th tour all the committed nodes from the i-th tour of the best so far solution
    			for (int j = 1; j <= index; j++) {
    				city = Ants.best_so_far_ant.tours.get(i).get(j);
    				distance = VRPTW.instance.distance[current_city][city + 1];
    		    	arrivalTime = a.currentTime.get(i) + reqList.get(current_city).getServiceTime() + distance; 
    		    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
    		    	if (beginService > reqList.get(city + 1).getEndWindow()) {
    		    		System.out.println("Method addCommitedNodes: solution infeasible..");
    		    	}
    		    	
    		    	//add committed node to the ant's tour
    		    	a.tours.get(i).add(j, city);
    				a.visited[city] = true; 
    				a.toVisit--;
    				a.currentTime.set(i, beginService);
    				a.beginService[city + 1] = beginService;
    				a.currentQuantity.set(i, a.currentQuantity.get(i) + reqList.get(city + 1).getDemand());
    				
    				current_city = city + 1;
    			}
    		}
    	}
    }
    
    //check if there are any committed tours (i.e. tours that contain at least one committed node that
    //should be included in the ant's solution)
    static boolean checkCommitedTours() {
    	boolean result = false;
    	int lastPos;
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}
    	
    	for (int index : lastCommitedIndexes) {
    		if (index > 0) {
    			return true;
    		}
    	}
    	
    	return result;
    }

    //manage the solution construction phase (construct a set of complete and closed tours, one
    //for each vehicle)
    static void construct_solutions(VRPTW instance) {
		int k; /* counter variable */
		int step; /* counter of the number of construction steps */
		int values[] = new int[2];
	
		/* Mark all cities as unvisited */
		for (k = 0; k < Ants.n_ants; k++) {
		    Ants.ant_empty_memory(Ants.ants[k], instance);
		}
	
		step = 0;
		/* Place the ants on same initial city, which is the depot city */
		for (k = 0; k < Ants.n_ants; k++) {
			//Ants.place_ant(Ants.ants[k], step);
			for (int i = 0; i < Ants.ants[k].usedVehicles; i++) {
				//place each ant on the depot city, which is the start city of each tour
				// -1 is a special marker for the deport city, so that it's not be confused with the rest of the cities
				// all the rest of the cities are represented by integer values > 0
				Ants.ants[k].tours.get(i).add(-1);   
			}
		}
		
		//initialize ant solution with the committed nodes (if any) from each tour of the best so far solution
		if (checkCommitedTours()) {
			for (k = 0; k < Ants.n_ants; k++) {
				addCommitedNodes(Ants.ants[k], instance);
				//System.out.println("After addCommitedNodes");
			}
		}
		
		while (!isDone()) {
		    for (k = 0; k < Ants.n_ants; k++) {
		    	if (Ants.ants[k].toVisit > 0) {
		    		//choose for each ant in a probabilistic way by some type of roullette wheel selection 
					//which salesman to consider next, that will visit a city
		    		//salesman = (int)(Math.random() * MTsp.m);
					values = Ants.neighbour_choose_and_move_to_next(Ants.ants[k], instance);
					if (values[0] != -1) {
						if (Ants.acs_flag)
							Ants.local_acs_pheromone_update(Ants.ants[k], values[1]);
					}

		    	}
		    	
		    }
		}
		
		//System.out.println("After building tours");
	
		double longestTourLength;
		int idLongestTour = 0;
		for (k = 0; k < Ants.n_ants; k++) {
			InOut.noSolutions++;
			longestTourLength = Double.MIN_VALUE;
			for (int i = 0; i < Ants.ants[k].usedVehicles; i++) {
				step = Ants.ants[k].tours.get(i).size();
				Ants.ants[k].tours.get(i).add(step, -1);
				
				Ants.ants[k].tour_lengths.set(i, VRPTW.compute_tour_length_(Ants.ants[k].tours.get(i)));
				Ants.ants[k].total_tour_length += Ants.ants[k].tour_lengths.get(i);
				if (longestTourLength < Ants.ants[k].tour_lengths.get(i)) {
					longestTourLength = Ants.ants[k].tour_lengths.get(i);
					idLongestTour = i;
				}
				
			    if (Ants.acs_flag)
			    	Ants.local_acs_pheromone_update(Ants.ants[k], i);
			}
			Ants.ants[k].longest_tour_length = longestTourLength;
			Ants.ants[k].indexLongestTour = idLongestTour;
			//Ants.ants[k].total_tour_length = Tsp.compute_tour_lengths(Ants.ants[k].tours);
			Ants.ants[k].costObjectives[0] = Ants.ants[k].total_tour_length;
			Ants.ants[k].costObjectives[1] = Ants.computeToursAmplitude(Ants.ants[k]);
		}
		InOut.n_tours += (Ants.n_ants * Ants.ants[0].usedVehicles); //each ant constructs a complete and closed tour
    }

    //initialize variables appropriately when starting a trial
    static void init_try(VRPTW instance) {

		Timer.start_timers();
		InOut.time_used = Timer.elapsed_time();
		InOut.time_passed = InOut.time_used;
		int noAvailableNodes;
	
		/* Initialize variables concerning statistics etc. */
		InOut.n_tours = 1;
		InOut.iteration = 1;
		Ants.best_so_far_ant.total_tour_length = Double.MAX_VALUE;
		Ants.restart_best_ant.total_tour_length = Double.MAX_VALUE;
		InOut.found_best = 0;
		InOut.lambda = 0.05;
		
		/*
		 * Initialize the Pheromone trails, only if ACS is used, Ants.pheromones
		 * have to be initialized differently
		 */
		if (!(Ants.acs_flag)) {
		    Ants.trail_0 = 1. / ((Ants.rho) * Ants.nn_tour(instance));
		    /*
		     * in the original papers on Ant System it is not exactly defined what the
		     * initial value of the Ants.pheromones is. Here we set it to some
		     * small constant, analogously as done in MAX-MIN Ant System.
		     */
		    Ants.init_pheromone_trails(Ants.trail_0);
		}
		if (Ants.acs_flag) {
			noAvailableNodes = instance.getIdAvailableRequests().size();
			//if no cities are available except the depot, set the pheromone levels to a static value
			if (noAvailableNodes == 0) {
				Ants.trail_0 = 1.0;
			}
			else {
				Ants.trail_0 = 1. / ((double) (noAvailableNodes + 1) * (double) Ants.nn_tour(instance));
			}
		    Ants.init_pheromone_trails(Ants.trail_0);
		}
	
		/* Calculate combined information Ants.pheromone times heuristic information */
		//Ants.compute_total_information();
		
    }
    
    //apply to each solution constructed by a ant, a local search phase to further improve the solution
    //the local search procedure is composed of 2 operators applied sequentially in this order: 
    //relocate single/multiple route and exchange single/multiple route
    //as a result, all ants of the colony will have locally optimal tours
    static void apply_local_search(VRPTW instance) {
    	for (int k = 0; k < Ants.n_ants; k++) {
    		Ants.ants[k] = local_search(Ants.ants[k], instance);
    	}
    }
    
    //skip committed (defined) nodes when applying local search operators 
    static Ant local_search(Ant a, VRPTW instance) {  	
		//apply relocate multiple route local search operator 
		a = relocateMultipleRoute(a, instance);  
		
		//apply exchange multiple route local search operator 
		a = exchangeMultipleRoute(a, instance);
		
		return a;
    }
    
	static boolean checkFeasibleTourRelocationMultiple(Ant a, VRPTW vrp, int indexTourSource, int indexTourDestination, int i, int j) {
		boolean isFeasible = true;
		int city, previousCity, prevCity, nextCity, currentCity;
		double currentQuantity, arrivalTime, currentTime = 0.0, beginService, earliestTime, latestTime, distance;
		ArrayList<Request> reqList = vrp.getRequests();
		double value1, value2, value3, value4;
		
		city = a.tours.get(indexTourSource).get(i);
		currentQuantity = a.currentQuantity.get(indexTourDestination) + reqList.get(city + 1).getDemand();
		if (currentQuantity > vrp.getCapacity()) {
			return false;
		}
		
		//check time window constraints in source tour
		for (int pos = i + 1; pos < a.tours.get(indexTourSource).size(); pos++) {
			if (pos == (i + 1)) {
				prevCity = a.tours.get(indexTourSource).get(pos - 2);
				currentCity = a.tours.get(indexTourSource).get(pos);
				currentTime = a.beginService[prevCity + 1];
			}
			else {
				prevCity = a.tours.get(indexTourSource).get(pos - 1);
        		currentCity = a.tours.get(indexTourSource).get(pos);
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		return false;
	    	}
	    	currentTime = beginService;
		}
		
		/*previousCity = a.tours.get(indexTourDestination).get(j - 1);
        nextCity = a.tours.get(indexTourDestination).get(j);
    	
    	arrivalTime = a.beginService[previousCity + 1] + reqList.get(previousCity + 1).getServiceTime() + VRPTW.instance.distance[previousCity + 1][city + 1]; 
    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
    	if (beginService > reqList.get(city + 1).getEndWindow()) {
    		return false;
    	}
    	
    	value1 = new Double(reqList.get(city + 1).getStartWindow());
    	//value2 = a.earliestTime.get(indexTour).get(previousPos) + VRPTW.instance.distance[previousCity + 1][customer + 1];
    	value2 = a.earliestTime.get(indexTourDestination).get(j - 1) + VRPTW.instance.distance[previousCity + 1][city + 1] + reqList.get(previousCity + 1).getServiceTime();
    	earliestTime = Math.max(value1, value2);
    	value3 = new Double(reqList.get(city + 1).getEndWindow());
    	//value4 = a.latestTime.get(indexTour).get(nextPos) - VRPTW.instance.distance[customer + 1][nextCity + 1];
    	value4 = a.latestTime.get(indexTourDestination).get(j) - VRPTW.instance.distance[city + 1][nextCity + 1] - reqList.get(city + 1).getServiceTime();
    	latestTime = Math.min(value3, value4);
    	
		if (earliestTime > latestTime) {
			return false;
		}*/
		
		previousCity = a.tours.get(indexTourDestination).get(j - 1);
        nextCity = a.tours.get(indexTourDestination).get(j);
    	
    	arrivalTime = a.beginService[previousCity + 1] + reqList.get(previousCity + 1).getServiceTime() + VRPTW.instance.distance[previousCity + 1][city + 1]; 
    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
    	if (beginService > reqList.get(city + 1).getEndWindow()) {
    		return false;
    	}
    	currentTime = beginService;
    	
    	arrivalTime = currentTime + reqList.get(city + 1).getServiceTime() + VRPTW.instance.distance[city + 1][nextCity + 1]; 
    	beginService = Math.max(arrivalTime, reqList.get(nextCity + 1).getStartWindow());
    	if (beginService > reqList.get(nextCity + 1).getEndWindow()) {
    		return false;
    	}
    	currentTime = beginService;
		
		//check time window constraints in destination tour
		for (int pos = j + 1; pos < a.tours.get(indexTourDestination).size(); pos++) {
			prevCity = a.tours.get(indexTourDestination).get(pos - 1);
    		currentCity = a.tours.get(indexTourDestination).get(pos);
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		return false;
	    	}
	    	currentTime = beginService;
		}
		
		return isFeasible;
	}
	
	static void updateBeginServiceRelocationMultiple(Ant a, VRPTW vrp, int indexTourSource, int indexTourDestination, int i, int j) {
		int currentCity, prevCity;
		double currentTime = 0.0;
		double distance, arrivalTime, beginService = 0.0;
		ArrayList<Request> reqList = vrp.getRequests();
		
		//update of begin service times for the source tour
		for (int pos = i; pos < a.tours.get(indexTourSource).size() - 1; pos++) {
			prevCity = a.tours.get(indexTourSource).get(pos - 1);
			currentCity = a.tours.get(indexTourSource).get(pos);
			if (pos == i) {
				currentTime = a.beginService[prevCity + 1];
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	currentTime = beginService;
	    	a.beginService[currentCity + 1] = beginService;	
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		System.out.println("Relocation Multiple indexTourSource: Unfeasible solution..");
	    	}
		}
		a.currentTime.set(indexTourSource, beginService);
		
		//update of begin service times for the destination tour
		for (int pos = j; pos < a.tours.get(indexTourDestination).size() - 1; pos++) {
			prevCity = a.tours.get(indexTourDestination).get(pos - 1);
			currentCity = a.tours.get(indexTourDestination).get(pos);
			if (pos == j) {
				currentTime = a.beginService[prevCity + 1];
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	currentTime = beginService;
	    	a.beginService[currentCity + 1] = beginService;	
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		System.out.println("Relocation Multiple indexTourDestination: Unfeasible solution..");
	    	}
		}
		a.currentTime.set(indexTourDestination, beginService);
		
	}
    
    //skip committed (defined) nodes
    //relocateMultipleRoute is performed only once and the best improvement (which minimizes most the
    //total traveled distance) is applied
    static Ant relocateMultipleRoute(Ant a, VRPTW instance) {
    	boolean feasible = false;
		int city, sourcePrevCity, sourceNextCity = -2, destinationPrevCity, destinationNextCity;
		int startIndexSource, startIndexDestination;
		double newQuantity1, newQuantity2, newDistance1, newDistance2, newTotalDistance;
		ArrayList<Request> reqList = instance.getRequests();
		int lastPos;
		
		Ant bestImprovedAnt = new Ant();
		bestImprovedAnt.tours = new ArrayList();
		bestImprovedAnt.tour_lengths = new ArrayList<Double>();
		bestImprovedAnt.beginService = new double[VRPTW.n + 1];
		bestImprovedAnt.currentTime = new ArrayList<Double>();
		bestImprovedAnt.currentQuantity = new ArrayList<Double>();
		bestImprovedAnt.usedVehicles = 1;
	    for (int j = 0; j < bestImprovedAnt.usedVehicles; j++) {
	    	bestImprovedAnt.tours.add(j, new ArrayList<Integer>());
	    	bestImprovedAnt.tour_lengths.add(j, 0.0);
	    }
	    bestImprovedAnt.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    bestImprovedAnt.toVisit = instance.getIdAvailableRequests().size();
	    bestImprovedAnt.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	bestImprovedAnt.costObjectives[indexObj] = 0;
    	}
	    bestImprovedAnt.earliestTime = new ArrayList(bestImprovedAnt.usedVehicles);
	    bestImprovedAnt.latestTime = new ArrayList(bestImprovedAnt.usedVehicles);
	    
	    Ant temp = new Ant();
	    temp.tours = new ArrayList();
	    temp.tour_lengths = new ArrayList<Double>();
	    temp.beginService = new double[VRPTW.n + 1];
	    temp.currentTime = new ArrayList<Double>();
	    temp.currentQuantity = new ArrayList<Double>();
	    temp.usedVehicles = 1;
	    for (int j = 0; j < temp.usedVehicles; j++) {
	    	temp.tours.add(j, new ArrayList<Integer>());
	    	temp.tour_lengths.add(j, 0.0);
	    }
	    temp.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    temp.toVisit = instance.getIdAvailableRequests().size();
	    temp.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	temp.costObjectives[indexObj] = 0;
    	}
	    temp.earliestTime = new ArrayList(temp.usedVehicles);
	    temp.latestTime = new ArrayList(temp.usedVehicles);
	    
    	Ants.copy_from_to(a, bestImprovedAnt, instance);
    	Ants.copy_from_to(a, temp, instance);
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}

    	//System.out.println("Iteration " + InOut.iteration + " Entering relocateMultipleRouteInsertion " + "foundInsertion:" + foundInsertion + " a.toVisit:" + a.toVisit + " toContinue:" + toContinue);
		for (int indexTourSource = 0; indexTourSource < a.usedVehicles; indexTourSource++) {
			for (int indexTourDestination = 0; indexTourDestination < a.usedVehicles; indexTourDestination++) {
				if (indexTourSource != indexTourDestination) {
					//index of the element to be moved/relocated
					if (indexTourSource > lastCommitedIndexes.size() - 1) {
						startIndexSource = 1;
					}
					else {
						startIndexSource = lastCommitedIndexes.get(indexTourSource) + 1;
					}
					
					//index of the relocation's destination
					if (indexTourDestination > lastCommitedIndexes.size() - 1) {
						startIndexDestination = 1;
					}
					else {
						startIndexDestination = lastCommitedIndexes.get(indexTourDestination) + 1;
					}
					for (int i = startIndexSource; i < a.tours.get(indexTourSource).size() - 1; i++) { 
						for (int j = startIndexDestination; j < a.tours.get(indexTourDestination).size(); j++) {
							//check if results a feasible solution (i.e. no time window constraint is violated)
							feasible = checkFeasibleTourRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
							if (feasible) {
								//obtain the neighbour solution corresponding to the relocation operator
								city = temp.tours.get(indexTourSource).get(i);
							    temp.tours.get(indexTourSource).remove(i);
							    temp.tours.get(indexTourDestination).add(j, city);
								
							    newQuantity1 = temp.currentQuantity.get(indexTourSource) - reqList.get(city + 1).getDemand();
							    temp.currentQuantity.set(indexTourSource, newQuantity1);
							    newQuantity2 = temp.currentQuantity.get(indexTourDestination) + reqList.get(city + 1).getDemand();
							    temp.currentQuantity.set(indexTourDestination, newQuantity2); 
							   
								//update the begin service times of the nodes from the source and destination tours of the obtained neighbour solution 
								//also update the current time of the source and destination tours
							    updateBeginServiceRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
	
								//update total traveled distance and lengths of source and destination tours 
								sourcePrevCity = temp.tours.get(indexTourSource).get(i - 1);					
								sourceNextCity = temp.tours.get(indexTourSource).get(i);
								newDistance1 = temp.tour_lengths.get(indexTourSource) - VRPTW.instance.distance[sourcePrevCity + 1][city + 1] - VRPTW.instance.distance[city + 1][sourceNextCity + 1] + VRPTW.instance.distance[sourcePrevCity + 1][sourceNextCity + 1];
								
								destinationPrevCity = temp.tours.get(indexTourDestination).get(j - 1);
								destinationNextCity = temp.tours.get(indexTourDestination).get(j + 1);
								newDistance2 = temp.tour_lengths.get(indexTourDestination) - VRPTW.instance.distance[destinationPrevCity + 1][destinationNextCity + 1] + VRPTW.instance.distance[destinationPrevCity + 1][city + 1] + VRPTW.instance.distance[city + 1][destinationNextCity + 1];
							
								newTotalDistance = temp.total_tour_length - temp.tour_lengths.get(indexTourSource) - temp.tour_lengths.get(indexTourDestination) + newDistance1 + newDistance2;
								temp.total_tour_length = newTotalDistance;
								temp.tour_lengths.set(indexTourSource, newDistance1);
								temp.tour_lengths.set(indexTourDestination, newDistance2);		
								
								//if the source tour becomes empty (no city is visited except the depot), remove this empty tour
								if (temp.tours.get(indexTourSource).size() == 2 && temp.tours.get(indexTourSource).get(0) == -1 && temp.tours.get(indexTourSource).get(1) == -1) {
									temp.tours.remove(indexTourSource);
									temp.usedVehicles--;
								}
								
								//if some improvement is obtained in the total traveled distance
								if ((temp.total_tour_length < bestImprovedAnt.total_tour_length) 
									|| (temp.usedVehicles < bestImprovedAnt.usedVehicles)) {
									Ants.copy_from_to(temp, bestImprovedAnt, instance);
								}
								
								//restore previous solution constructed by ant
								Ants.copy_from_to(a, temp, instance);
							}
							
						}
					}
				}
				
				
			}
	    }

    	return bestImprovedAnt;
    }
    
    //find the index of the tour having the smallest number of visited nodes (cities/customers)
    static int findShortestTour(Ant a) {
    	int indexTour = 0;
    	int min = Integer.MAX_VALUE;
    	
    	for (int i = 0; i < a.usedVehicles; i++) {
    		if (min > a.tours.get(i).size()) {
    			min = a.tours.get(i).size();
    			indexTour = i;
    		}
    	}
    	
    	return indexTour;
    	
    }
    
    //skip committed (defined) nodes
    //relocateMultipleRouteIterated is performed multiple times until no further improvement (which minimizes most the
    //total traveled distance or reduces the number of used vehicles) is possible
    static Ant relocateMultipleRouteIterated(Ant a, VRPTW instance) {
    	boolean feasible = false;
		int city, sourcePrevCity, sourceNextCity = -2, destinationPrevCity, destinationNextCity;
		int startIndexSource, startIndexDestination;
		double newQuantity1, newQuantity2, newDistance1, newDistance2, newTotalDistance;
		ArrayList<Request> reqList = instance.getRequests();
		boolean foundImprovement = true, isValid;
		int lastPos;
		double tempNo = Math.pow(10, 10);
		double round1, round2;
		
		Ant improvedAnt = new Ant();
		improvedAnt.tours = new ArrayList();
		improvedAnt.tour_lengths = new ArrayList<Double>();
		improvedAnt.beginService = new double[VRPTW.n + 1];
		improvedAnt.currentTime = new ArrayList<Double>();
		improvedAnt.currentQuantity = new ArrayList<Double>();
		improvedAnt.usedVehicles = 1;
	    for (int j = 0; j < improvedAnt.usedVehicles; j++) {
	    	improvedAnt.tours.add(j, new ArrayList<Integer>());
	    	improvedAnt.tour_lengths.add(j, 0.0);
	    }
	    improvedAnt.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    improvedAnt.toVisit = instance.getIdAvailableRequests().size();
	    improvedAnt.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	improvedAnt.costObjectives[indexObj] = 0;
    	}
	    improvedAnt.earliestTime = new ArrayList(improvedAnt.usedVehicles);
	    improvedAnt.latestTime = new ArrayList(improvedAnt.usedVehicles);
	    
	    Ant temp = new Ant();
	    temp.tours = new ArrayList();
	    temp.tour_lengths = new ArrayList<Double>();
	    temp.beginService = new double[VRPTW.n + 1];
	    temp.currentTime = new ArrayList<Double>();
	    temp.currentQuantity = new ArrayList<Double>();
	    temp.usedVehicles = 1;
	    for (int j = 0; j < temp.usedVehicles; j++) {
	    	temp.tours.add(j, new ArrayList<Integer>());
	    	temp.tour_lengths.add(j, 0.0);
	    }
	    temp.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    temp.toVisit = instance.getIdAvailableRequests().size();
	    temp.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	temp.costObjectives[indexObj] = 0;
    	}
	    temp.earliestTime = new ArrayList(temp.usedVehicles);
	    temp.latestTime = new ArrayList(temp.usedVehicles);
	    
    	Ants.copy_from_to(a, improvedAnt, instance);
    	Ants.copy_from_to(a, temp, instance);
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}
    	
    	int count = 0;
    	while (foundImprovement) {
    		//System.out.println("foundImprovement = " + foundImprovement + " count = " + count);
    		foundImprovement = false;
    		
    		count++;
    		if (count > 100) {
    			System.out.println("Inside relocateMultipleRouteIterated; count=" + count);
    		}
    		
    		Ants.copy_from_to(improvedAnt, a, instance);
    		Ants.copy_from_to(improvedAnt, temp, instance);
    		
    		for (int indexTourSource = 0; indexTourSource < temp.usedVehicles; indexTourSource++) {
				for (int indexTourDestination = 0; indexTourDestination < temp.usedVehicles; indexTourDestination++) {
					if (indexTourSource != indexTourDestination) {
						//index of the element to be moved/relocated
						if (indexTourSource > lastCommitedIndexes.size() - 1) {
							startIndexSource = 1;
						}
						else {
							startIndexSource = lastCommitedIndexes.get(indexTourSource) + 1;
						}
						
						//index of the relocation's destination
						if (indexTourDestination > lastCommitedIndexes.size() - 1) {
							startIndexDestination = 1;
						}
						else {
							startIndexDestination = lastCommitedIndexes.get(indexTourDestination) + 1;
						}
						for (int i = startIndexSource; i < temp.tours.get(indexTourSource).size() - 1; i++) { 
							for (int j = startIndexDestination; j < temp.tours.get(indexTourDestination).size(); j++) {
								//check if results a feasible solution (i.e. no time window constraint is violated)
								feasible = checkFeasibleTourRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
								if (feasible) {
									//obtain the neighbour solution corresponding to the relocation operator
									city = temp.tours.get(indexTourSource).get(i);
								    temp.tours.get(indexTourSource).remove(i);
								    temp.tours.get(indexTourDestination).add(j, city);
								    
								    /*isValid = Utilities.checkFeasibility(temp, instance, false);
						    	    if (!isValid) {
						    	    	System.out.println("Inside relocateMultipleRouteIterated: The resulted solution is not valid (feasible)..");
						    	    }*/
									
								    newQuantity1 = temp.currentQuantity.get(indexTourSource) - reqList.get(city + 1).getDemand();
								    temp.currentQuantity.set(indexTourSource, newQuantity1);
								    newQuantity2 = temp.currentQuantity.get(indexTourDestination) + reqList.get(city + 1).getDemand();
								    temp.currentQuantity.set(indexTourDestination, newQuantity2); 
								   
									//update the begin service times of the nodes from the source and destination tours of the obtained neighbour solution 
									//also update the current time of the source and destination tours
								    updateBeginServiceRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
		
									//update total traveled distance and lengths of source and destination tours 
									sourcePrevCity = temp.tours.get(indexTourSource).get(i - 1);					
									sourceNextCity = temp.tours.get(indexTourSource).get(i);
									newDistance1 = temp.tour_lengths.get(indexTourSource) - VRPTW.instance.distance[sourcePrevCity + 1][city + 1] - VRPTW.instance.distance[city + 1][sourceNextCity + 1] + VRPTW.instance.distance[sourcePrevCity + 1][sourceNextCity + 1];
									
									destinationPrevCity = temp.tours.get(indexTourDestination).get(j - 1);
									destinationNextCity = temp.tours.get(indexTourDestination).get(j + 1);
									newDistance2 = temp.tour_lengths.get(indexTourDestination) - VRPTW.instance.distance[destinationPrevCity + 1][destinationNextCity + 1] + VRPTW.instance.distance[destinationPrevCity + 1][city + 1] + VRPTW.instance.distance[city + 1][destinationNextCity + 1];
								
									newTotalDistance = temp.total_tour_length - temp.tour_lengths.get(indexTourSource) - temp.tour_lengths.get(indexTourDestination) + newDistance1 + newDistance2;
									temp.total_tour_length = newTotalDistance;
									temp.tour_lengths.set(indexTourSource, newDistance1);
									temp.tour_lengths.set(indexTourDestination, newDistance2);		
									
									//if the source tour becomes empty (no city is visited except the depot), remove this empty tour
									if (temp.tours.get(indexTourSource).size() == 2 && temp.tours.get(indexTourSource).get(0) == -1 && temp.tours.get(indexTourSource).get(1) == -1) {
										//System.out.println("Reducing with 1 the number of used vehicles");
										temp.tours.remove(indexTourSource);
										temp.tour_lengths.remove(indexTourSource);
										temp.currentQuantity.remove(indexTourSource);
										temp.currentTime.remove(indexTourSource);
										temp.usedVehicles--;
									}
									
									//performing the rounding of the two numbers up to 10 decimals so that in the
									//comparison of the 2 double values to consider only the first 10 most significant decimals
									round1 = Math.round(temp.total_tour_length * tempNo) / tempNo;
									round2 = Math.round(improvedAnt.total_tour_length * tempNo) / tempNo;
									//if some improvement is obtained in the total traveled distance
									if (((round1 < round2) && (temp.usedVehicles == improvedAnt.usedVehicles))
											|| (temp.usedVehicles < improvedAnt.usedVehicles)) {
										Ants.copy_from_to(temp, improvedAnt, instance);
										foundImprovement = true;
									}
									
									//restore previous solution constructed by ant
									Ants.copy_from_to(a, temp, instance);
								}
								
							}
						}
				}
				
				}
				
			 }
	    }

    	return improvedAnt;
    }
    
    //skip committed (defined) nodes
    //relocateMultipleRouteIterated is performed multiple times until no further improvement (which minimizes most the
    //total traveled distance or reduces the number of used vehicles) is possible
    //try to relocate nodes from the tour having the minimum number of visited nodes, in an attempt
    //to reduce (minimize) the number of tours (i.e. used vehicles)
    static Ant relocateMultipleRouteIteratedShortest(Ant a, VRPTW instance) {
    	boolean feasible = false;
		int city, sourcePrevCity, sourceNextCity = -2, destinationPrevCity, destinationNextCity;
		int startIndexSource, startIndexDestination;
		double newQuantity1, newQuantity2, newDistance1, newDistance2, newTotalDistance;
		ArrayList<Request> reqList = instance.getRequests();
		boolean foundImprovement = true;
		int indexTourSource, lastPos;
		
		Ant improvedAnt = new Ant();
		improvedAnt.tours = new ArrayList();
		improvedAnt.tour_lengths = new ArrayList<Double>();
		improvedAnt.beginService = new double[VRPTW.n + 1];
		improvedAnt.currentTime = new ArrayList<Double>();
		improvedAnt.currentQuantity = new ArrayList<Double>();
		improvedAnt.usedVehicles = 1;
	    for (int j = 0; j < improvedAnt.usedVehicles; j++) {
	    	improvedAnt.tours.add(j, new ArrayList<Integer>());
	    	improvedAnt.tour_lengths.add(j, 0.0);
	    }
	    improvedAnt.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    improvedAnt.toVisit = instance.getIdAvailableRequests().size();
	    improvedAnt.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	improvedAnt.costObjectives[indexObj] = 0;
    	}
	    improvedAnt.earliestTime = new ArrayList(improvedAnt.usedVehicles);
	    improvedAnt.latestTime = new ArrayList(improvedAnt.usedVehicles);
	    
	    Ant temp = new Ant();
	    temp.tours = new ArrayList();
	    temp.tour_lengths = new ArrayList<Double>();
	    temp.beginService = new double[VRPTW.n + 1];
	    temp.currentTime = new ArrayList<Double>();
	    temp.currentQuantity = new ArrayList<Double>();
	    temp.usedVehicles = 1;
	    for (int j = 0; j < temp.usedVehicles; j++) {
	    	temp.tours.add(j, new ArrayList<Integer>());
	    	temp.tour_lengths.add(j, 0.0);
	    }
	    temp.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    temp.toVisit = instance.getIdAvailableRequests().size();
	    temp.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	temp.costObjectives[indexObj] = 0;
    	}
	    temp.earliestTime = new ArrayList(temp.usedVehicles);
	    temp.latestTime = new ArrayList(temp.usedVehicles);
	    
    	Ants.copy_from_to(a, improvedAnt, instance);
    	Ants.copy_from_to(a, temp, instance);
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}
    	
    	while (foundImprovement) {
    		foundImprovement = false;
    		
    		Ants.copy_from_to(improvedAnt, a, instance);
    		Ants.copy_from_to(a, temp, instance);
    		
    		indexTourSource = findShortestTour(a);
			for (int indexTourDestination = 0; indexTourDestination < a.usedVehicles; indexTourDestination++) {
				if (indexTourSource != indexTourDestination) {
					//index of the element to be moved/relocated
					if (indexTourSource > lastCommitedIndexes.size() - 1) {
						startIndexSource = 1;
					}
					else {
						startIndexSource = lastCommitedIndexes.get(indexTourSource) + 1;
					}
					
					//index of the relocation's destination
					if (indexTourDestination > lastCommitedIndexes.size() - 1) {
						startIndexDestination = 1;
					}
					else {
						startIndexDestination = lastCommitedIndexes.get(indexTourDestination) + 1;
					}
					for (int i = startIndexSource; i < a.tours.get(indexTourSource).size() - 1; i++) { 
						for (int j = startIndexDestination; j < a.tours.get(indexTourDestination).size(); j++) {
							//check if results a feasible solution (i.e. no time window constraint is violated)
							feasible = checkFeasibleTourRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
							if (feasible) {
								//obtain the neighbour solution corresponding to the relocation operator
								city = temp.tours.get(indexTourSource).get(i);
							    temp.tours.get(indexTourSource).remove(i);
							    temp.tours.get(indexTourDestination).add(j, city);
								
							    newQuantity1 = temp.currentQuantity.get(indexTourSource) - reqList.get(city + 1).getDemand();
							    temp.currentQuantity.set(indexTourSource, newQuantity1);
							    newQuantity2 = temp.currentQuantity.get(indexTourDestination) + reqList.get(city + 1).getDemand();
							    temp.currentQuantity.set(indexTourDestination, newQuantity2); 
							   
								//update the begin service times of the nodes from the source and destination tours of the obtained neighbour solution 
								//also update the current time of the source and destination tours
							    updateBeginServiceRelocationMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
	
								//update total traveled distance and lengths of source and destination tours 
								sourcePrevCity = temp.tours.get(indexTourSource).get(i - 1);					
								sourceNextCity = temp.tours.get(indexTourSource).get(i);
								newDistance1 = temp.tour_lengths.get(indexTourSource) - VRPTW.instance.distance[sourcePrevCity + 1][city + 1] - VRPTW.instance.distance[city + 1][sourceNextCity + 1] + VRPTW.instance.distance[sourcePrevCity + 1][sourceNextCity + 1];
								
								destinationPrevCity = temp.tours.get(indexTourDestination).get(j - 1);
								destinationNextCity = temp.tours.get(indexTourDestination).get(j + 1);
								newDistance2 = temp.tour_lengths.get(indexTourDestination) - VRPTW.instance.distance[destinationPrevCity + 1][destinationNextCity + 1] + VRPTW.instance.distance[destinationPrevCity + 1][city + 1] + VRPTW.instance.distance[city + 1][destinationNextCity + 1];
							
								newTotalDistance = temp.total_tour_length - temp.tour_lengths.get(indexTourSource) - temp.tour_lengths.get(indexTourDestination) + newDistance1 + newDistance2;
								temp.total_tour_length = newTotalDistance;
								temp.tour_lengths.set(indexTourSource, newDistance1);
								temp.tour_lengths.set(indexTourDestination, newDistance2);		
								
								//if the source tour becomes empty (no city is visited except the depot), remove this empty tour
								if (temp.tours.get(indexTourSource).size() == 2 && temp.tours.get(indexTourSource).get(0) == -1 && temp.tours.get(indexTourSource).get(1) == -1) {
									temp.tours.remove(indexTourSource);
									temp.usedVehicles--;
								}
								
								//if some improvement is obtained in the total traveled distance
								if (((temp.total_tour_length < improvedAnt.total_tour_length) && (temp.usedVehicles == improvedAnt.usedVehicles))
									|| (temp.usedVehicles < improvedAnt.usedVehicles)) {
									Ants.copy_from_to(temp, improvedAnt, instance);
									foundImprovement = true;
								}
								
								//restore previous solution constructed by ant
								Ants.copy_from_to(a, temp, instance);
							}
							
						}
					}
			}
			
			
		 }
	    }

    	return improvedAnt;
    }
    
    static boolean checkFeasibleTourExchangeMultiple(Ant a, VRPTW vrp, int indexTourSource, int indexTourDestination, int i, int j) {
    	boolean isFeasible = true;
		int currentCity, prevCity, city1, city2;
		double currentTime = 0.0;
		double distance, arrivalTime, beginService, currentQuantity;
		ArrayList<Request> reqList = vrp.getRequests();
		
		//check vehicle capacity tour constraints for source and destination tours
		city1 = a.tours.get(indexTourSource).get(i);
		city2 = a.tours.get(indexTourDestination).get(j);
		currentQuantity = a.currentQuantity.get(indexTourSource) - reqList.get(city1 + 1).getDemand() + reqList.get(city2 + 1).getDemand();
		if (currentQuantity > vrp.getCapacity()) {
			return false;
		}
		currentQuantity = a.currentQuantity.get(indexTourDestination) - reqList.get(city2 + 1).getDemand() + reqList.get(city1 + 1).getDemand();
		if (currentQuantity > vrp.getCapacity()) {
			return false;
		}
		
		//check feasibility for source tour regarding time windows constraints
		for (int pos = i; pos < a.tours.get(indexTourSource).size(); pos++) {
			if (pos == i) {
				prevCity = a.tours.get(indexTourSource).get(pos - 1);
				currentCity = a.tours.get(indexTourDestination).get(j);
				currentTime = a.beginService[prevCity + 1];
			}
			else if (pos == (i + 1)) {
				prevCity = a.tours.get(indexTourDestination).get(j);
				currentCity = a.tours.get(indexTourSource).get(pos);
			}
			else {
				prevCity = a.tours.get(indexTourSource).get(pos - 1);
        		currentCity = a.tours.get(indexTourSource).get(pos);
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		return false;
	    	}
	    	currentTime = beginService;
		}
		
		//check feasibility for destination tour regarding time windows constraints
		for (int pos = j; pos < a.tours.get(indexTourDestination).size(); pos++) {
			if (pos == j) {
				prevCity = a.tours.get(indexTourDestination).get(pos - 1);
				currentCity = a.tours.get(indexTourSource).get(i);
				currentTime = a.beginService[prevCity + 1];
			}
			else if (pos == (j + 1)) {
				prevCity = a.tours.get(indexTourSource).get(i);
				currentCity = a.tours.get(indexTourDestination).get(pos);
			}
			else {
				prevCity = a.tours.get(indexTourDestination).get(pos - 1);
        		currentCity = a.tours.get(indexTourDestination).get(pos);
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		return false;
	    	}
	    	currentTime = beginService;
		}
		
		return isFeasible;
    	
	}
    
    static void updateBeginServiceExchangeMultiple(Ant a, VRPTW vrp, int indexTourSource, int indexTourDestination, int i, int j) {
		int currentCity, prevCity;
		double currentTime = 0.0;
		double distance, arrivalTime, beginService = 0.0;
		ArrayList<Request> reqList = vrp.getRequests();
		
		for (int pos = i; pos < a.tours.get(indexTourSource).size() - 1; pos++) {
			prevCity = a.tours.get(indexTourSource).get(pos - 1);
			currentCity = a.tours.get(indexTourSource).get(pos);
			if (pos == i) {
				currentTime = a.beginService[prevCity + 1];
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	currentTime = beginService;
	    	a.beginService[currentCity + 1] = beginService;
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		System.out.println("Exchange Multiple indexTourSource: Unfeasible solution..");
	    	}
		}
		a.currentTime.set(indexTourSource, beginService);
		
		for (int pos = j; pos < a.tours.get(indexTourDestination).size() - 1; pos++) {
			prevCity = a.tours.get(indexTourDestination).get(pos - 1);
			currentCity = a.tours.get(indexTourDestination).get(pos);
			if (pos == j) {
				currentTime = a.beginService[prevCity + 1];
			}
			distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
	    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
	    	currentTime = beginService;
	    	a.beginService[currentCity + 1] = beginService;
	    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
	    		System.out.println("Exchange Multiple indexTourDestination: Unfeasible solution..");
	    	}
		}
		a.currentTime.set(indexTourDestination, beginService);
		
	}
    
    static Ant exchangeMultipleRoute(Ant a, VRPTW instance) {
    	boolean feasible = false;
		int city1, city2, sourcePrevCity, sourceNextCity = -2, destinationPrevCity, destinationNextCity;
		int startIndexSource, startIndexDestination;
		double newQuantity1, newQuantity2, newDistance1, newDistance2, newTotalDistance;
		ArrayList<Request> reqList = instance.getRequests();
		int lastPos;
		
		Ant bestImprovedAnt = new Ant();
		bestImprovedAnt.tours = new ArrayList();
		bestImprovedAnt.tour_lengths = new ArrayList<Double>();
		bestImprovedAnt.beginService = new double[VRPTW.n + 1];
		bestImprovedAnt.currentTime = new ArrayList<Double>();
		bestImprovedAnt.currentQuantity = new ArrayList<Double>();
		bestImprovedAnt.usedVehicles = 1;
	    for (int j = 0; j < bestImprovedAnt.usedVehicles; j++) {
	    	bestImprovedAnt.tours.add(j, new ArrayList<Integer>());
	    	bestImprovedAnt.tour_lengths.add(j, 0.0);
	    }
	    bestImprovedAnt.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    bestImprovedAnt.toVisit = instance.getIdAvailableRequests().size();
	    bestImprovedAnt.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	bestImprovedAnt.costObjectives[indexObj] = 0;
    	}
	    bestImprovedAnt.earliestTime = new ArrayList(bestImprovedAnt.usedVehicles);
	    bestImprovedAnt.latestTime = new ArrayList(bestImprovedAnt.usedVehicles);
	    
	    Ant temp = new Ant();
	    temp.tours = new ArrayList();
	    temp.tour_lengths = new ArrayList<Double>();
	    temp.beginService = new double[VRPTW.n + 1];
	    temp.currentTime = new ArrayList<Double>();
	    temp.currentQuantity = new ArrayList<Double>();
	    temp.usedVehicles = 1;
	    for (int j = 0; j < temp.usedVehicles; j++) {
	    	temp.tours.add(j, new ArrayList<Integer>());
	    	temp.tour_lengths.add(j, 0.0);
	    }
	    temp.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    temp.toVisit = instance.getIdAvailableRequests().size();
	    temp.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	temp.costObjectives[indexObj] = 0;
    	}
	    temp.earliestTime = new ArrayList(temp.usedVehicles);
	    temp.latestTime = new ArrayList(temp.usedVehicles);
	    
    	Ants.copy_from_to(a, bestImprovedAnt, instance);
    	Ants.copy_from_to(a, temp, instance);
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}
    	
		for (int indexTourSource = 0; indexTourSource < (a.usedVehicles - 1); indexTourSource++) {
			for (int indexTourDestination = indexTourSource + 1; indexTourDestination < a.usedVehicles; indexTourDestination++) {
				if (indexTourSource != indexTourDestination) {
					//index of the element to be moved from the source tour
					if (indexTourSource > lastCommitedIndexes.size() - 1) {
						startIndexSource = 1;
					}
					else {
						startIndexSource = lastCommitedIndexes.get(indexTourSource) + 1;
					}
					
					//index of the element to be moved from the destination tour
					if (indexTourDestination > lastCommitedIndexes.size() - 1) {
						startIndexDestination = 1;
					}
					else {
						startIndexDestination = lastCommitedIndexes.get(indexTourDestination) + 1;
					}
					for (int i = startIndexSource; i < a.tours.get(indexTourSource).size() - 1; i++) { 
						for (int j = startIndexDestination; j < a.tours.get(indexTourDestination).size() - 1; j++) {
							if (indexTourSource <= indexTourDestination) {
								//check if results a feasible solution (i.e. no time window constraint is violated)
								feasible = checkFeasibleTourExchangeMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
								if (feasible) {
									//obtain the neighbour solution corresponding to the relocation operator
									city1 = temp.tours.get(indexTourSource).get(i);
									city2 = temp.tours.get(indexTourDestination).get(j);
								    temp.tours.get(indexTourSource).set(i, city2);
								    temp.tours.get(indexTourDestination).set(j, city1);
									
								    newQuantity1 = temp.currentQuantity.get(indexTourSource) - reqList.get(city1 + 1).getDemand() + reqList.get(city2 + 1).getDemand();
								    temp.currentQuantity.set(indexTourSource, newQuantity1);
								    newQuantity2 = temp.currentQuantity.get(indexTourDestination) - reqList.get(city2 + 1).getDemand() + reqList.get(city1 + 1).getDemand();
								    temp.currentQuantity.set(indexTourDestination, newQuantity2); 
								   
								    //update the begin service times of the nodes from the source and destination tours of the obtained neighbour solution 
									//also update the current time of the source and destination tours
									updateBeginServiceExchangeMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
								
									//update total traveled distance and lengths of source and destination tours 
									sourcePrevCity = temp.tours.get(indexTourSource).get(i - 1);					
									sourceNextCity = temp.tours.get(indexTourSource).get(i + 1);
									newDistance1 = temp.tour_lengths.get(indexTourSource) - VRPTW.instance.distance[sourcePrevCity + 1][city1 + 1] - VRPTW.instance.distance[city1 + 1][sourceNextCity + 1] + VRPTW.instance.distance[sourcePrevCity + 1][city2 + 1] + VRPTW.instance.distance[city2 + 1][sourceNextCity + 1];
									
									destinationPrevCity = temp.tours.get(indexTourDestination).get(j - 1);
									destinationNextCity = temp.tours.get(indexTourDestination).get(j + 1);
									newDistance2 = temp.tour_lengths.get(indexTourDestination) - VRPTW.instance.distance[destinationPrevCity + 1][city2 + 1] - VRPTW.instance.distance[city2 + 1][destinationNextCity + 1] + VRPTW.instance.distance[destinationPrevCity + 1][city1 + 1] + VRPTW.instance.distance[city1 + 1][destinationNextCity + 1];
								
									newTotalDistance = temp.total_tour_length - temp.tour_lengths.get(indexTourSource) - temp.tour_lengths.get(indexTourDestination) + newDistance1 + newDistance2;
									temp.total_tour_length = newTotalDistance;
									temp.tour_lengths.set(indexTourSource, newDistance1);
									temp.tour_lengths.set(indexTourDestination, newDistance2);	
							
									//if some improvement is obtained in the total traveled distance
									if (temp.total_tour_length < bestImprovedAnt.total_tour_length) {
										Ants.copy_from_to(temp, bestImprovedAnt, instance);
									}
									
									//restore previous solution constructed by ant
									Ants.copy_from_to(a, temp, instance);
								}
							}
							
						}
					}
				}
				
			}
	    }
    	
    	return bestImprovedAnt;
    }
    
    static Ant exchangeMultipleRouteIterated(Ant a, VRPTW instance) {
    	boolean feasible = false;
		int city1, city2, sourcePrevCity, sourceNextCity = -2, destinationPrevCity, destinationNextCity;
		int startIndexSource, startIndexDestination;
		double newQuantity1, newQuantity2, newDistance1, newDistance2, newTotalDistance;
		ArrayList<Request> reqList = instance.getRequests();
		boolean foundImprovement = true, isValid;
		int lastPos;
		double tempNo = Math.pow(10, 10);
		double round1, round2;
		
		Ant improvedAnt = new Ant();
		improvedAnt.tours = new ArrayList();
		improvedAnt.tour_lengths = new ArrayList<Double>();
		improvedAnt.beginService = new double[VRPTW.n + 1];
		improvedAnt.currentTime = new ArrayList<Double>();
		improvedAnt.currentQuantity = new ArrayList<Double>();
		improvedAnt.usedVehicles = 1;
	    for (int j = 0; j < improvedAnt.usedVehicles; j++) {
	    	improvedAnt.tours.add(j, new ArrayList<Integer>());
	    	improvedAnt.tour_lengths.add(j, 0.0);
	    }
	    improvedAnt.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    improvedAnt.toVisit = instance.getIdAvailableRequests().size();
	    improvedAnt.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	improvedAnt.costObjectives[indexObj] = 0;
    	}
	    improvedAnt.earliestTime = new ArrayList(improvedAnt.usedVehicles);
	    improvedAnt.latestTime = new ArrayList(improvedAnt.usedVehicles);
	    
	    Ant temp = new Ant();
	    temp.tours = new ArrayList();
	    temp.tour_lengths = new ArrayList<Double>();
	    temp.beginService = new double[VRPTW.n + 1];
	    temp.currentTime = new ArrayList<Double>();
	    temp.currentQuantity = new ArrayList<Double>();
	    temp.usedVehicles = 1;
	    for (int j = 0; j < temp.usedVehicles; j++) {
	    	temp.tours.add(j, new ArrayList<Integer>());
	    	temp.tour_lengths.add(j, 0.0);
	    }
	    temp.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
	    temp.toVisit = instance.getIdAvailableRequests().size();
	    temp.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	temp.costObjectives[indexObj] = 0;
    	}
	    temp.earliestTime = new ArrayList(temp.usedVehicles);
	    temp.latestTime = new ArrayList(temp.usedVehicles);
	    
    	Ants.copy_from_to(a, improvedAnt, instance);
    	Ants.copy_from_to(a, temp, instance);
    	
    	ArrayList<Integer> lastCommitedIndexes = new  ArrayList<Integer>();
		for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
			lastPos = Controller.getLastCommitedPos(index);
			lastCommitedIndexes.add(lastPos);
		}
    	
		int count = 0;
    	while (foundImprovement) {
    		foundImprovement = false;
    		
    		count++;
    		if (count > 100) {
    			System.out.println("Inside exchangeMultipleRouteIterated; count=" + count);
    		}
    		
    		Ants.copy_from_to(improvedAnt, a, instance);
    		Ants.copy_from_to(improvedAnt, temp, instance);
    		
			for (int indexTourSource = 0; indexTourSource < (temp.usedVehicles - 1); indexTourSource++) {
				for (int indexTourDestination = indexTourSource + 1; indexTourDestination < temp.usedVehicles; indexTourDestination++) {
					if (indexTourSource != indexTourDestination) {
						//index of the element to be moved from the source tour
						if (indexTourSource > lastCommitedIndexes.size() - 1) {
							startIndexSource = 1;
						}
						else {
							startIndexSource = lastCommitedIndexes.get(indexTourSource) + 1;
						}
						
						//index of the element to be moved from the destination tour
						if (indexTourDestination > lastCommitedIndexes.size() - 1) {
							startIndexDestination = 1;
						}
						else {
							startIndexDestination = lastCommitedIndexes.get(indexTourDestination) + 1;
						}
						for (int i = startIndexSource; i < temp.tours.get(indexTourSource).size() - 1; i++) { 
							for (int j = startIndexDestination; j < temp.tours.get(indexTourDestination).size() - 1; j++) {
								if (indexTourSource <= indexTourDestination) {
									//check if results a feasible solution (i.e. no time window constraint is violated)
									feasible = checkFeasibleTourExchangeMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
									if (feasible) {
										//obtain the neighbour solution corresponding to the relocation operator
										city1 = temp.tours.get(indexTourSource).get(i);
										city2 = temp.tours.get(indexTourDestination).get(j);
									    temp.tours.get(indexTourSource).set(i, city2);
									    temp.tours.get(indexTourDestination).set(j, city1);
									    
									    /*isValid = Utilities.checkFeasibility(temp, instance, false);
							    	    if (!isValid) {
							    	    	System.out.println("Inside exchangeMultipleRouteIterated: The resulted solution is not valid (feasible)..");
							    	    }*/
										
									    newQuantity1 = temp.currentQuantity.get(indexTourSource) - reqList.get(city1 + 1).getDemand() + reqList.get(city2 + 1).getDemand();
									    temp.currentQuantity.set(indexTourSource, newQuantity1);
									    newQuantity2 = temp.currentQuantity.get(indexTourDestination) - reqList.get(city2 + 1).getDemand() + reqList.get(city1 + 1).getDemand();
									    temp.currentQuantity.set(indexTourDestination, newQuantity2); 
									   
									    //update the begin service times of the nodes from the source and destination tours of the obtained neighbour solution 
										//also update the current time of the source and destination tours
										updateBeginServiceExchangeMultiple(temp, instance, indexTourSource, indexTourDestination, i, j);
									
										//update total traveled distance and lengths of source and destination tours 
										sourcePrevCity = temp.tours.get(indexTourSource).get(i - 1);					
										sourceNextCity = temp.tours.get(indexTourSource).get(i + 1);
										newDistance1 = temp.tour_lengths.get(indexTourSource) - VRPTW.instance.distance[sourcePrevCity + 1][city1 + 1] - VRPTW.instance.distance[city1 + 1][sourceNextCity + 1] + VRPTW.instance.distance[sourcePrevCity + 1][city2 + 1] + VRPTW.instance.distance[city2 + 1][sourceNextCity + 1];
										
										destinationPrevCity = temp.tours.get(indexTourDestination).get(j - 1);
										destinationNextCity = temp.tours.get(indexTourDestination).get(j + 1);
										newDistance2 = temp.tour_lengths.get(indexTourDestination) - VRPTW.instance.distance[destinationPrevCity + 1][city2 + 1] - VRPTW.instance.distance[city2 + 1][destinationNextCity + 1] + VRPTW.instance.distance[destinationPrevCity + 1][city1 + 1] + VRPTW.instance.distance[city1 + 1][destinationNextCity + 1];
									
										newTotalDistance = temp.total_tour_length - temp.tour_lengths.get(indexTourSource) - temp.tour_lengths.get(indexTourDestination) + newDistance1 + newDistance2;
										temp.total_tour_length = newTotalDistance;
										temp.tour_lengths.set(indexTourSource, newDistance1);
										temp.tour_lengths.set(indexTourDestination, newDistance2);	
								
										//performing the rounding of the two numbers up to 10 decimals so that in the
										//comparison of the 2 double values to consider only the first 10 most significant decimals
										round1 = Math.round(temp.total_tour_length * tempNo) / tempNo;
										round2 = Math.round(improvedAnt.total_tour_length * tempNo) / tempNo;		
										//if some improvement is obtained in the total traveled distance
										if (round1 < round2) {
											Ants.copy_from_to(temp, improvedAnt, instance);
											foundImprovement = true;
										}
										
										//restore previous solution constructed by ant
										Ants.copy_from_to(a, temp, instance);
									}
								}
								
							}
						}
					}
					
				}
		    }
    	}
    	
    	return improvedAnt;
    }
    
    //manage some statistical information about the trial, especially if a new best solution
    //(best-so-far) is found and adjust some parameters if a new best solution is found
    static void update_statistics(VRPTW instance) {
		int iteration_best_ant;
		double scalingValue, scalledValue = 0.0, sum;
		Object obj = new Object();
		double round1, round2;
		double tempNo = Math.pow(10, 10);
	
		iteration_best_ant = Ants.find_best(); /* iteration_best_ant is a global variable */
		
		//Ants.ants[iteration_best_ant] = local_search(Ants.ants[iteration_best_ant]);
		
		Ant a = Ants.ants[iteration_best_ant]; 
		
		//System.out.println("Entering update_statistics");
		
		boolean isValid; 
		if (ls_flag) {
			/*isValid = Utilities.checkFeasibility(a, instance, false);
    	    if (!isValid) {
    	    	System.out.println("Before relocateMultipleRouteIterated: The resulted solution is not valid (feasible)..");
    	    }*/
			//apply multiple route relocation and exchange iterated local search operators for the iteration best ant
        	a = relocateMultipleRouteIterated(a, instance);
        	/*isValid = Utilities.checkFeasibility(a, instance, false);
    	    if (!isValid) {
    	    	System.out.println("After relocateMultipleRouteIterated: The resulted solution is not valid (feasible)..");
    	    }*/
        	//System.out.println("Exited relocateMultipleRouteIterated");
        	a = exchangeMultipleRouteIterated(a, instance);	
        	//System.out.println("Exited exchangeMultipleRouteIterated");
        	/*isValid = Utilities.checkFeasibility(a, instance, false);
    	    if (!isValid) {
    	    	System.out.println("After exchangeMultipleRouteIterated: The resulted solution is not valid (feasible)..");
    	    }*/
		}
		
		synchronized (obj) {
			round1 = Math.round(a.total_tour_length * tempNo) / tempNo;
			round2 = Math.round(Ants.best_so_far_ant.total_tour_length * tempNo) / tempNo;
			if ((a.usedVehicles < Ants.best_so_far_ant.usedVehicles) || ((a.usedVehicles == Ants.best_so_far_ant.usedVehicles) && (round1 < round2))
				|| ((round1 < round2) && (Ants.best_so_far_ant.total_tour_length == Double.MAX_VALUE))) {
			
			    InOut.time_used = Timer.elapsed_time();  //best solution found after time_used 
			    /*if (a.usedVehicles < Ants.best_so_far_ant.usedVehicles) {
			    	for (int i = a.usedVehicles; i < Ants.best_so_far_ant.usedVehicles; i++) {
			    		Ants.lastCommitted.remove(a.usedVehicles);
			    	}
			    }*/
			    Ants.copy_from_to(a, Ants.best_so_far_ant, instance);
	    
			    scalingValue = Controller.getScalingValue();
				if (scalingValue != 0) {
					scalledValue = Ants.best_so_far_ant.total_tour_length / scalingValue;
				}
			    /*System.out.println("Updated Best so far ant >> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length + " (scalled value = " + scalledValue + ")");
			    for (int i = 0; i < Ants.best_so_far_ant.usedVehicles; i++) {
					int tourLength = Ants.best_so_far_ant.tours.get(i).size();
					for (int j = 0; j < tourLength; j++) {
						int city = Ants.best_so_far_ant.tours.get(i).get(j);
						//only for Unix shell prompts: print committed nodes with red color at console
						if (city >= 0) {
							if (Ants.committedNodes[city]) {
								System.out.print(Utilities.ANSI_RED);
							}
						}
						
						//so as to correspond to the city indexes from the VRPTW input file
						System.out.print((city + 1) + " ");	
						//reset color	
						if (city >= 0) {
							if (Ants.committedNodes[city]) {
								System.out.print(Utilities.ANSI_RESET);
							}
						}
					}
					System.out.println();
					//System.out.print(Utilities.ANSI_RESET);
				}*/
			    
			    /*isValid = Utilities.checkFeasibility(Ants.best_so_far_ant, instance, true);
	    	    if (!isValid) {
	    	    	System.out.println("Inside update_statistics: The solution is not valid (feasible)..");
	    	    }
	    	    else if (isValid) {
	    	    	System.out.println("Inside update_statistics: The solution is valid (feasible)..");
	    	    }*/
			}
		}
		
    }
    
    //manage some statistical information about the trial, especially if a new best solution
    //(best-so-far) is found and adjust some parameters if a new best solution is found
    static void update_statistics(boolean saveIterCosts, VRPTW instance) {
		int iteration_best_ant;
	
		iteration_best_ant = Ants.find_best(); /* iteration_best_ant is a global variable */
		
		//Ants.ants[iteration_best_ant] = local_search(Ants.ants[iteration_best_ant]);
		
		Ant a = Ants.ants[iteration_best_ant];
		
		//a = relocationIterated(a);  
		
		/*if (ls_flag) {
			//apply exchange local search operator for the iteration best ant
			a = exchangeIterated(a, instance); 
			
			//apply 2-opt heuristic on each of the m routes; 2-opt tries to improve single route
			//by replacing its two non-adjacent edges by two other edges
			for (int l = 0; l < a.usedVehicles; l++) {
				LocalSearch.two_opt_first(a.tours.get(l));
			}	
			
			//compute new distances and update longest tour
			double sum = 0.0;
			double longestTourLength = Double.MIN_VALUE;
			int idLongestTour = 0;
			for (int l = 0; l < a.usedVehicles; l++) {
				a.tour_lengths.set(l, VRPTW.compute_tour_length(a.tours.get(l)));
				sum += a.tour_lengths.get(l);
				if (longestTourLength < a.tour_lengths.get(l)) {
					longestTourLength = a.tour_lengths.get(l);
					idLongestTour = l;
				}
			}
			a.total_tour_length = sum;
			a.longest_tour_length = longestTourLength;
			a.indexLongestTour = idLongestTour;
		}*/
		
		if ((a.usedVehicles < Ants.best_so_far_ant.usedVehicles) || (a.usedVehicles == Ants.best_so_far_ant.usedVehicles) && (a.total_tour_length < Ants.best_so_far_ant.total_tour_length)
				|| ((a.total_tour_length < Ants.best_so_far_ant.total_tour_length) && (Ants.best_so_far_ant.total_tour_length == Double.MAX_VALUE))) {
			
		    InOut.time_used = Timer.elapsed_time();  //best solution found after time_used 
		    Ants.copy_from_to(a, Ants.best_so_far_ant, instance);
		    Ants.copy_from_to(a, Ants.restart_best_ant, instance);
	
		    InOut.found_best = InOut.iteration;	 
		    InOut.restart_found_best = InOut.iteration;
		    //InOut.branching_factor = InOut.node_branching(InOut.lambda);
		    //System.out.println("Iter: " + InOut.iteration + " Best ant -> longest tour=" + Ants.best_so_far_ant.longest_tour_length + ", b_fac " + InOut.branching_factor);
		    //System.out.println("Iter: " + InOut.iteration + " Best ant -> longest tour=" + Ants.best_so_far_ant.longest_tour_length);
		    //System.out.println("Iter: " + InOut.iteration + " Best ant >> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length);
		}
		
		if ((a.usedVehicles < Ants.restart_best_ant.usedVehicles) || (a.usedVehicles == Ants.restart_best_ant.usedVehicles) && (a.total_tour_length < Ants.restart_best_ant.total_tour_length)
				|| ((a.total_tour_length < Ants.restart_best_ant.total_tour_length) && (Ants.restart_best_ant.total_tour_length == Double.MAX_VALUE))) {
		    Ants.copy_from_to(a, Ants.restart_best_ant, instance);
		    
		    InOut.restart_found_best = InOut.iteration;
		    //System.out.println("Iter: " + InOut.iteration + " Restart best ant >> No. of used vehicles=" + Ants.restart_best_ant.usedVehicles + " total tours length=" + Ants.restart_best_ant.total_tour_length);
		}
		
		/*if (!saveIterCosts) {
			//compute non-dominated set of solutions (iteration non-dominated front)
			ParetoFront.iterationPareto.clear();
			Ant copyAnt;
			for (int i = 0; i < Ants.n_ants; i++) {
				copyAnt = Ants.copyAnt(Ants.ants[i]);
				ParetoFront.paretoUpdateWithSolution(ParetoFront.iterationPareto, copyAnt);
			}
			
			//update BestSoFarPareto external set
			ParetoFront.paretoUpdate(ParetoFront.bestSoFarPareto, ParetoFront.iterationPareto);
		}*/
	
		
    }

    //occasionally compute some statistics
   //at every 5 iterations save the value of the longest cost of the solution/tour given by the best so far ant
    static void search_control_and_statistics()
    {
    	double longestCost;
    	double initTrail;
    	
    	/*if (saveDetailedOutput) {
    		if ((InOut.iteration % 5) == 0) {
			    //System.out.println("TSP(" + tspIndex + "): best tour length so far " + Ants.best_so_far_ant[tspIndex].tour_length + ", iteration: " + InOut.iteration);
	    		longestCost = Ants.best_so_far_ant.total_tour_length;
	    		
	    		iterLongestCost.add(longestCost);
	    		if (trial == 0) {
	    			iterNumber.add(InOut.iteration);
	    		}
    		}
    	}*/
    	
    	/*if ((InOut.iteration % 5) == 0) {
    	    InOut.branching_factor = InOut.node_branching(InOut.lambda);
    	    //System.out.println("Iter: " + InOut.iteration + ", b_fac " + InOut.branching_factor);

    	    //if (InOut.iteration - InOut.restart_found_best >= 200) {
    	    if (((InOut.branching_factor <= InOut.branch_fac) && (InOut.iteration - InOut.restart_found_best > 100)) || ((InOut.iteration - InOut.restart_found_best >= 300))) {
    			// Ants.pheromone trail re-initialisation takes place
    			//System.out.println("Reinitialisation of pheromone trails...");
    			Ants.restart_best_ant.total_tour_length = Double.MAX_VALUE;
    			initTrail = 1. / ((double) (VRPTW.n + 1) * (Ants.best_so_far_ant.total_tour_length * Ants.best_so_far_ant.usedVehicles));
    			Ants.init_pheromone_trails(initTrail);
    	    }
    	}*/
    }
    
    //occasionally compute some statistics
   //at every 5 iterations save the value of the longest cost of the solution/tour given by the best so far ant
    static void search_control_and_statistics(ArrayList<Double> iterLongestCost, ArrayList<Integer> iterNumber, boolean saveDetailedOutput, int trial)
    {
    	double longestCost;
    	double initTrail;
    	
    	if (saveDetailedOutput) {
    		if ((InOut.iteration % 5) == 0) {
			    //System.out.println("TSP(" + tspIndex + "): best tour length so far " + Ants.best_so_far_ant[tspIndex].tour_length + ", iteration: " + InOut.iteration);
	    		longestCost = Ants.best_so_far_ant.total_tour_length;
	    		
	    		iterLongestCost.add(longestCost);
	    		if (trial == 0) {
	    			iterNumber.add(InOut.iteration);
	    		}
    		}
    	}
    	
    	/*if ((InOut.iteration % 5) == 0) {
    	    InOut.branching_factor = InOut.node_branching(InOut.lambda);
    	    //System.out.println("Iter: " + InOut.iteration + ", b_fac " + InOut.branching_factor);

    	    //if (InOut.iteration - InOut.restart_found_best >= 200) {
    	    if (((InOut.branching_factor <= InOut.branch_fac) && (InOut.iteration - InOut.restart_found_best > 100)) || ((InOut.iteration - InOut.restart_found_best >= 300))) {
    			// Ants.pheromone trail re-initialisation takes place
    			//System.out.println("Reinitialisation of pheromone trails...");
    			Ants.restart_best_ant.total_tour_length = Double.MAX_VALUE;
    			initTrail = 1. / ((double) (VRPTW.n + 1) * (Ants.best_so_far_ant.total_tour_length * Ants.best_so_far_ant.usedVehicles));
    			Ants.init_pheromone_trails(initTrail);
    	    }
    	}*/
    }

    //manage global Ants.pheromone deposit for Ant System
    static void as_update() {
		int k;
	
		for (k = 0; k < Ants.n_ants; k++)
		    Ants.global_update_pheromone(Ants.ants[k]);
    }

    //manage global Ants.pheromone deposit for Ant Colony System
    static void acs_global_update() {
    	Ants.global_acs_pheromone_update(Ants.best_so_far_ant);
    	
    	/*if (InOut.iteration - InOut.restart_found_best > 100) {
    		//System.out.println("Using best so far ant for global pheromone update");
    		Ants.global_acs_pheromone_update(Ants.best_so_far_ant);
    	}
    	else {
    		//System.out.println("Using restart best ant for global pheromone update");
    		Ants.global_acs_pheromone_update(Ants.restart_best_ant);
    	}*/
    	
    }

    //manage global Ants.pheromone trail update for the ACO algorithms
	static void pheromone_trail_update()
	{
		//Simulate the Ants.pheromone evaporation of all Ants.pheromones; this is not necessary for ACS
		if (Ants.as_flag) {
			/* evaporate all Ants.pheromone trails */
			Ants.evaporation();
		}
	
		/* Next, apply the Ants.pheromone deposit for the various ACO algorithms */
		if (Ants.as_flag)
		    as_update();
		else if (Ants.acs_flag)
		    acs_global_update();
	
	
		/*
		 * Compute combined information Ants.pheromone times heuristic info after
		 * the Ants.pheromone update for all ACO algorithms except ACS; in the ACS case
		 * this is already done in the Ants.pheromone update procedures of ACS
		 */
		if (Ants.as_flag) {
			//Ants.compute_total_information();
	    }
	}
	
	//do the work of running the ant colony which tries to solve the DVRPTW instance using the available
	//(known so far) customer requests
	public void run() {
		int counter = 0;
		isRunning = true;
		
		//System.out.println("Starting ant colony thread; isRunning=" + isRunning);
	    while (isRunning) {
	    	//the thread was restarted (i.e. it was previously stopped and started again)
	    	if (threadRestarted) {
	    		//compute the new value for the initial pheromone trail based on the current best solution so far
	    		int noAvailableNodes = vrpInstance.getIdAvailableRequests().size();
	    		Ants.trail_0 = 1. / ((double) (noAvailableNodes + 1) * (double) Ants.best_so_far_ant.total_tour_length);
	    		
	    		//preserve a certain amount of the pheromones from the previous run of the ant colony
	    		Ants.preservePheromones(vrpInstance);
	    	}
	    	//do the optimization task (work)
	    	construct_solutions(vrpInstance);
	    	//increase evaluations counter
	    	InOut.noEvaluations++;
	    	counter++;

	    	
	    	/*if (ls_flag) {
				apply_local_search(vrpInstance);
			}	*/	 
	    	update_statistics(vrpInstance);
			pheromone_trail_update();
			search_control_and_statistics();
			//force the ant colony thread to stop its execution
			if (counter > 300) {
				isRunning = false;
			}
	    	 
	    }
	    //System.out.println("Counter=" + counter + "; After terminating work in thread: best solution is >> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length);
	}

	public void terminate() {
		isRunning = false;
	}	
	
	public static void main(String[] args) {
		boolean saveIterCosts = false;
		//long startCPUTime, endCPUTime;
		
		for (int trial = 0; trial < 1; trial++) {
			long startTime = System.currentTimeMillis();
			
			/*ThreadMXBean bean = ManagementFactory.getThreadMXBean();
			startCPUTime = bean.isCurrentThreadCpuTimeSupported() ? bean.getCurrentThreadCpuTime() : 0L;*/
			
	    	//read the data from the input file
			DataReader reader = new DataReader(Controller.vrpInstance);
	        //read the data from the file
	        VRPTW vrpInstance = reader.read();
	        
	        System.out.println("\nDVRPTW_ACS MinSum>> Solving dynamic VRPTW instance: " + VRPTW.instance.name);
	        System.out.println("No. of customers' requests (except the depot): " +  VRPTW.n);
	        //System.out.println("No. of desired tours (used vehicles): " + VRPTW.m);
	        
	        //Utilities.writeInputDataPoints();
	        
			/*Request depot = vrpInstance.getRequests().get(0);
			System.out.println("Request corresponding to depot:"); 
			System.out.println("XCoord: " + depot.getxCoord() + " YCoord: " + depot.getyCoord() + 
			 " demand: " + depot.getDemand() + " startWindow: " + depot.getStartWindow() + " endWindow: " + depot.getEndWindow() +
			 " service time: " + depot.getServiceTime());*/
			
			//System.out.println("Sorted list of requests..");
			/*int size = vrpInstance.getRequests().size();
			ArrayList<Request>copyRequests = new ArrayList<Request>(size);
			for (int i = 1; i < size; i++) {
				copyRequests.add(vrpInstance.getRequests().get(i));
			}
			Collections.sort(copyRequests);*/
			/*Request req;
			for (int i = 0; i < copyRequests.size(); i++) {
				req = copyRequests.get(i);
				System.out.println("ID: " + req.getId() + " XCoord: " + req.getxCoord() + " YCoord: " + req.getyCoord() + 
						 " demand: " + req.getDemand() + " startWindow: " + req.getStartWindow() + " endWindow: " + req.getEndWindow() +
						 " service time: " + req.getServiceTime());
			}*/
			
			/*Request req;
			for (int i = 0; i < vrpInstance.getRequests().size(); i++) {
				req = vrpInstance.getRequests().get(i);
				System.out.println("ID: " + req.getId() + " XCoord: " + req.getxCoord() + " YCoord: " + req.getyCoord() + 
						 " demand: " + req.getDemand() + " startWindow: " + req.getStartWindow() + " endWindow: " + req.getEndWindow() +
						 " service time: " + req.getServiceTime());
			}*/
			
	      
			InOut.init_program(args, trial, vrpInstance, 0.0);
			
			int[][][] result = new int[2][][];
			result = VRPTW.compute_nn_lists(vrpInstance);
			VRPTW.instance.nn_list = result[0];
			VRPTW.instance.nn_list_all = result[1];
			
			Ants.pheromone = new double[VRPTW.n + 1][VRPTW.n + 1];
			//Ants.total = new double[MTsp.n + 1][MTsp.n + 1];
	
			generateInitialWeights();
			init_try(vrpInstance); 
	
     		ArrayList<Double> iterLongestCost = null;
			ArrayList<Integer> iterNumber = null;
			if (saveIterCosts) {
				//for saving detailed cost values at each 5 iterations
				iterLongestCost = new ArrayList<Double>();
				iterNumber = new ArrayList<Integer>();
			}
			
			InOut.iteration = 0;
		    while (!termination_condition()) {
				construct_solutions(vrpInstance);
				/*if (ls_flag) {
					apply_local_search();
				}	*/	    
				update_statistics(saveIterCosts, vrpInstance);
				pheromone_trail_update();
				search_control_and_statistics(iterLongestCost, iterNumber, saveIterCosts, trial);
				InOut.iteration++;
		    }
		     
		    /*Utilities.setIterLongestCost(iterLongestCost);
		    Utilities.setIterNumber(iterNumber);*/
		    
		   InOut.exit_try(trial);  
		   /* if (!saveIterCosts) {
		    	Utilities.writeParetoSet(ParetoFront.bestSoFarPareto, trial);
			    //System.out.println("Reached " + InOut.iteration + " iterations");
			    //Utilities.writeExcel(MTsp.n, MTsp.m, totalLength);
		    	Utilities.writeParetoSolutions(ParetoFront.bestSoFarPareto);
			    ParetoFront.bestSoFarPareto.clear();
		    }*/
		    // Utilities.writeResultsExcel(trial, saveIterCosts);
		    
		  long endTime = System.currentTimeMillis();
		  //endCPUTime = bean.isCurrentThreadCpuTimeSupported() ? bean.getCurrentThreadCpuTime() : 0L;
		  
		  double difference1 = (endTime - startTime)/1000.0;
		  System.out.println("Run #" + (trial + 1)  + " Elapsed seconds: " + difference1 + " Ellapsed iterations: " + InOut.iteration);   
		  
		  /*double difference2 = (endCPUTime - startCPUTime)/1000000000.0;
		  System.out.println("\nRun #" + (trial + 1)  + " Elapsed seconds (CPU time): " + difference2); */ 
		}
		
    }
}
