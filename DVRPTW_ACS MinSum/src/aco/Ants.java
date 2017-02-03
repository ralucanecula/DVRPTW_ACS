package aco;

import java.util.ArrayList;

/**
 * ACO algorithms for the TSP
 * 
 * This code is based on the ACOTSP project of Thomas Stuetzle.
 * It was initially ported from C to Java by Adrian Wilke.
 * 
 * Project website: http://adibaba.github.io/ACOTSPJava/
 * Source code: https://github.com/adibaba/ACOTSPJava/
 */
public class Ants {
	/*
     * ################################################
     * ########## ACO algorithms for the TSP ##########
     * ################################################
     * 
     * Version: 1.0
     * File: ants.c
     * Author: Thomas Stuetzle
     * Purpose: implementation of procedures for ants' behaviour
     * Check: README.txt and legal.txt
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
	
    static class Ant {
    	//for each of the m salesmen an ant will construct a tour, so that a candidate solution constructed by
    	//an ant will be represented by a list of tours, one for each salesman
    	ArrayList<ArrayList<Integer>> tours;
		boolean[] visited;
		ArrayList<Double> tour_lengths;
		//contains the beginning of service for each customer, including the depot, taking into 
		//account the opening time (beginning) of the time window, the service time and the 
		//distance between customers
		double[] beginService;
		//current number of used vehicles (= number of routes/tours) for constructing a feasible solution
		//that serve all the customers' requests and satisfy the time windows and capacity constraints
		int usedVehicles;
		//stores for each partial tour/route under construction the current time 
		//taking into account the beginning time of service for the last visited customer on the tour
		ArrayList<Double> currentTime;
		//used by the insertion heuristic when checking the feasibility of an insertion
		//it stores for every customer i already assigned to a route the earliest time a delivery 
		//can be made at i
		ArrayList<ArrayList<Double>> earliestTime;
		//used by the insertion heuristic when checking the feasibility of an insertion
		//it stores for every customer i already assigned to a route the latest time a delivery
		//can be made at i
		ArrayList<ArrayList<Double>> latestTime;
		//stores for each partial tour/route under construction the current quantity of goods 
		//(given by the demand of each request) transported on the route 
		ArrayList<Double> currentQuantity;
		double total_tour_length;
		double longest_tour_length;
		int indexLongestTour;  //the index of the longest tour
		//cities left to be visited by an ant (initially toVisit = n, which is the number of cities from the mTSP instance)
		int toVisit;
		//stores the cost of each solution according to the considered objectives (2 in this case)
		double costObjectives[];
		//it is true if a new empty tour was added in the ant solution to service the remaining available
		//unrouted/unvisited customers
		boolean addedEmptyTour;
    }

    public static final int MAX_ANTS = 1024;
    public static final int MAX_NEIGHBOURS = 512;
    
    public static final double weight1 = 0.4;  //0.3   //0.4
    public static final double weight2 = 0.4;  //0.5   //0.4
    public static final double weight3 = 0.2;  //0.2   //0.2
    
    //weight used in the nearest neighbour heuristic, when computing an initial solution to calculate the
    //value of the initial pheromone trail
    public static double initialWeight1;  
    public static double initialWeight2;  
    public static double initialWeight3;  
    
    //it indicates that node at position/index i is committed if the value at position i is true; the 
    //depot node is considered to be committed by default and it's not included in this array
    public volatile static boolean[] committedNodes;
    
    //for each tour in T* (best so far solution) it indicates the position/index of the last node that was committed
    //public volatile static ArrayList<Integer> lastCommitted;
    
    static Ant ants[];
    public volatile static Ant best_so_far_ant;
    static Ant restart_best_ant;

    static double pheromone[][];
    //static double total[][];  //keeps heuristic information times pheromone for each arc

    static double prob_of_selection[][];

    static int n_ants; /* number of ants */
 
    static int nn_ants; /* length of nearest neighbor lists for the ants' solution construction */

    static double rho; /* parameter for evaporation used in global pheromne update*/
    static double local_rho;  /* parameter for evaporation used in local pheromone update*/
    static double alpha; /* importance of trail */
    static double beta; /* importance of heuristic evaluate */
    static double q_0; /* probability of best choice in tour construction */

    static boolean as_flag; /* ant system */
    static boolean acs_flag; /* ant colony system (ACS) */

    static int u_gb; /* every u_gb iterations update with best-so-far ant */

    static double trail_0; /* initial pheromone level in ACS */

    
    static double HEURISTIC(int m, int n) {
    	return (1.0 / (double) VRPTW.instance.distance[m][n]);
    }

    //allocate the memory for the ant colony, the best-so-far ant
    static void allocate_ants(VRPTW instance) {
		int i, j;
	
		ants = new Ant[n_ants];
		
	    committedNodes = new boolean[VRPTW.n];
	    for (i = 0; i < VRPTW.n; i++) {
	    	committedNodes[i] = false;
	    }
	    //lastCommitted = new ArrayList<Integer>();
	
		for (i = 0; i < n_ants; i++) {
		    ants[i] = new Ant();
		    ants[i].tours = new ArrayList();
		    ants[i].tour_lengths = new ArrayList<Double>();
		    ants[i].beginService = new double[VRPTW.n + 1];
		    ants[i].currentTime = new ArrayList<Double>();
		    ants[i].currentQuantity = new ArrayList<Double>();
		    ants[i].usedVehicles = 1;
		    ants[i].addedEmptyTour = false;
		    for (j = 0; j < ants[i].usedVehicles; j++) {
		    	ants[i].tours.add(j, new ArrayList<Integer>());
		    	ants[i].tour_lengths.add(j, 0.0);
		    	//lastCommitted.add(j, 0); 
		    }
		    ants[i].visited = new boolean[VRPTW.n];
			//the another node is the depot, which is by default visited by each salesman and added in its tour
		    ants[i].toVisit = instance.getIdAvailableRequests().size();
		    ants[i].costObjectives = new double[2];
		    for (int indexObj = 0; indexObj < 2; indexObj++) {
		    	ants[i].costObjectives[indexObj] = 0;
	    	}
		    ants[i].earliestTime = new ArrayList(ants[i].usedVehicles);
		    ants[i].latestTime = new ArrayList(ants[i].usedVehicles);
		}
		
		best_so_far_ant = new Ant();
		best_so_far_ant.tours = new ArrayList();
		best_so_far_ant.tour_lengths = new ArrayList<Double>();
		best_so_far_ant.beginService = new double[VRPTW.n + 1];
		best_so_far_ant.currentTime = new ArrayList<Double>();
		best_so_far_ant.currentQuantity = new ArrayList<Double>();
		best_so_far_ant.usedVehicles = 1;
		best_so_far_ant.addedEmptyTour = false;
	    for (j = 0; j < best_so_far_ant.usedVehicles; j++) {
	    	best_so_far_ant.tours.add(j, new ArrayList<Integer>());
	    	best_so_far_ant.tour_lengths.add(j, 0.0);
	    }
		best_so_far_ant.visited = new boolean[VRPTW.n];
		//the another node is the depot, which is by default visited by each salesman and added in its tour
		best_so_far_ant.toVisit = instance.getIdAvailableRequests().size();
		best_so_far_ant.longest_tour_length = Double.MAX_VALUE;
		
		best_so_far_ant.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	best_so_far_ant.costObjectives[indexObj] = 0;
    	}
	    best_so_far_ant.earliestTime = new ArrayList(best_so_far_ant.usedVehicles);
	    best_so_far_ant.latestTime = new ArrayList(best_so_far_ant.usedVehicles);
	    
	    restart_best_ant = new Ant();
	    restart_best_ant.tours = new ArrayList();
	    restart_best_ant.tour_lengths = new ArrayList<Double>();
	    restart_best_ant.beginService = new double[VRPTW.n + 1];
	    restart_best_ant.currentTime = new ArrayList<Double>();
	    restart_best_ant.currentQuantity = new ArrayList<Double>();
	    restart_best_ant.usedVehicles = 1;
	    restart_best_ant.addedEmptyTour = false;
	    for (j = 0; j < restart_best_ant.usedVehicles; j++) {
	    	restart_best_ant.tours.add(j, new ArrayList<Integer>());
	    	restart_best_ant.tour_lengths.add(j, 0.0);
	    }
	    restart_best_ant.visited = new boolean[VRPTW.n];
	    //the another node is the depot, which is by default visited by each salesman and added in its tour
	    restart_best_ant.toVisit = instance.getIdAvailableRequests().size();
	    restart_best_ant.longest_tour_length = Double.MAX_VALUE;
		
	    restart_best_ant.costObjectives = new double[2];
	    for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	restart_best_ant.costObjectives[indexObj] = 0;
    	}
	    restart_best_ant.earliestTime = new ArrayList(restart_best_ant.usedVehicles);
	    restart_best_ant.latestTime = new ArrayList(restart_best_ant.usedVehicles);
	
		/*prob_of_selection = new double[ants[0].usedVehicles][nn_ants + 1];
		for (j = 0; j < ants[0].usedVehicles; j++) {
			for (i = 0; i < nn_ants; i++) {
			    prob_of_selection[j][i] = Double.POSITIVE_INFINITY;
			}
		}
		for (j = 0; j < (ants[0].usedVehicles - 1); j++) {
			prob_of_selection[j][nn_ants] = 0;
		}
		prob_of_selection[ants[0].usedVehicles - 1][nn_ants] = Double.POSITIVE_INFINITY;*/
    }

    //find the best ant of the current iteration (the one with the lowest number of used vehicles and
    //with smaller total traveled distance)
    static int find_best()
    {
    	double min1, min2;
		int k1, k2, k2_min;
		
		//first detect the ant which uses the minimum number of vehicles
		min1 = ants[0].usedVehicles;
		for (k1 = 1; k1 < n_ants; k1++) {
		    if (ants[k1].usedVehicles < min1) {
				min1 = ants[k1].usedVehicles;
		    }
		}
		
	    //among the vehicles which use the minimum number of vehicles, select the best ant as the one with the minimum total distance for its traveled tours
		min2 = Double.MAX_VALUE;
		k2_min = 0;
		for (k2 = 0; k2 < n_ants; k2++) {
			if (ants[k2].usedVehicles == min1) {
				if (ants[k2].total_tour_length < min2) {
					min2 = ants[k2].total_tour_length;
					k2_min = k2;
			    }
			}
		    
		}
		return k2_min;
    }

    //initialize pheromone trails
    //matricea cu urmele de feromoni trebuie sa se faca relativ la toate cele n orase
    static void init_pheromone_trails(double initial_trail)
    {
		int i, j;
	
		/* Initialize pheromone trails */
		for (i = 0; i < (VRPTW.n + 1); i++) {
		    for (j = 0; j <= i; j++) {
				pheromone[i][j] = initial_trail;
				pheromone[j][i] = initial_trail;
				/*total[i][j] = initial_trail;
				total[j][i] = initial_trail;*/
		    }
		}
    }
    
    //preserve some of the pheromone level on the edges between available nodes 
    static void preservePheromones(VRPTW vrp) {
    	for (int i = 0; i < (VRPTW.n + 1); i++) {
		    for (int j = 0; j <= i; j++) {
		    	if (vrp.getIdAvailableRequests().contains(i - 1) && vrp.getIdAvailableRequests().contains(j - 1)
		    	|| ((i == 0) && vrp.getIdAvailableRequests().contains(j - 1)) 
		    	|| ((j == 0) && vrp.getIdAvailableRequests().contains(i - 1)))	{
		    		pheromone[i][j] = pheromone[i][j] * (1 - InOut.pheromonePreservation) + InOut.pheromonePreservation * trail_0;
		    		pheromone[j][i] = pheromone[i][j];	
		    	}
		    	else {
		    		pheromone[i][j] = trail_0;
		    		pheromone[j][i] = pheromone[i][j];	
		    	}
		    }
    	}
    }

    //implements the pheromone trail evaporation
    static void evaporation()
    {
		int i, j;
		
		for (i = 0; i < VRPTW.n + 1; i++) {
		    for (j = 0; j <= i; j++) {
				pheromone[i][j] = (1 - rho) * pheromone[i][j];
				pheromone[j][i] = pheromone[i][j];
		    }
		}
    }

    //reinforces edges used in ant k's solution
    static void global_update_pheromone(Ant a)
    {
		int i, j, h, k, size;
		double d_tau;
	
		d_tau = 1.0 / (double) a.total_tour_length;
		for (i = 0; i < a.usedVehicles; i++) {
			size = a.tours.get(i).size();
			for (k = 0; k < size - 1; k++) {
			    j = a.tours.get(i).get(k); 
			    h = a.tours.get(i).get(k + 1);
			    
			    j++;
	            h++;
	            
			    pheromone[j][h] += d_tau;
			    pheromone[h][j] = pheromone[j][h];
			}
		}
		
    }

    //calculates heuristic info times pheromone for each arc
   /* static void compute_total_information()
    {
		int i, j;
	
		for (i = 0; i < MTsp.n + 1; i++) {
		    for (j = 0; j < i; j++) {
				total[i][j] = Math.pow(pheromone[i][j], alpha) * Math.pow(HEURISTIC(i, j), beta);
				total[j][i] = total[i][j];
		    }
		}
    }*/

    //empty the ants's memory regarding visited cities
    static void ant_empty_memory(Ant a, VRPTW instance)
    {
        int i, j;

		a.total_tour_length = 0;
		a.longest_tour_length = Integer.MAX_VALUE;
		a.indexLongestTour = 0;
		a.addedEmptyTour = false;
		
		for (int indexObj = 0; indexObj < 2; indexObj++) {
	    	a.costObjectives[indexObj] = 0;
	    }
		
		 a.tour_lengths.clear();
		 a.currentQuantity.clear();
		 a.currentTime.clear();
		
		//clear all the elements (cities) from the tours of an ant
		for (i = 0; i < a.usedVehicles; i++) {
		    a.tours.get(i).clear();
		    /*if (a.earliestTime != null && a.earliestTime.size() > i && a.earliestTime.get(i) != null) {
		    	a.earliestTime.get(i).clear();
		    }
		    if (a.latestTime != null && a.latestTime.size() > i && a.latestTime.get(i) != null) {
		    	a.latestTime.get(i).clear();
		    }*/
		    
		}
		a.tours.clear();
		if (a.earliestTime != null) {
			a.earliestTime.clear();
		}
		if (a.latestTime != null) {
			a.latestTime.clear();
		}
		
		a.usedVehicles = 1;
		
		for (i = 0; i < a.usedVehicles; i++) {
		    a.tour_lengths.add(i, 0.0);
		    a.currentQuantity.add(i, 0.0);
		    a.currentTime.add(i, 0.0);
		    a.tours.add(i, new ArrayList<Integer>());
		   /* a.earliestTime.add(i, new ArrayList<Double>());
		    a.latestTime.add(i, new ArrayList<Double>());*/
		}
		
		for (j = 0; j < VRPTW.n; j++) {
		    a.visited[j] = false;
		}
		for (j = 0; j < (VRPTW.n + 1); j++) {
		    a.beginService[j] = 0;
		}
		//the another node is the depot, which is by default visited by each salesman and added in its tour
		a.toVisit = instance.getIdAvailableRequests().size();
		
    }
    
    //create a copy of the ant a and return the created copy at the output
    static Ant copyAnt(Ant a) {
    	//first create an empty ant
    	//TODO: copy also fields such as usedVehicle, current quantity, currentTime
    	Ant copy = new Ant();
    	copy.tours = new ArrayList(a.usedVehicles);
    	copy.tour_lengths = new ArrayList<Double>(a.usedVehicles);
	    /*for (int j = 0; j < a.usedVehicles; j++) {
	    	copy.tours.add(j, new ArrayList<Integer>());
	    	copy.tour_lengths.add(j, 0.0);
	    }*/
	    copy.visited = new boolean[VRPTW.n];
	    copy.toVisit = VRPTW.n;
	    
	    copy.costObjectives = new double[2];
	    //copy.weights = new double[TSP_ACO.k];
    	
	    //then copy the information from the ant a
	    copy.total_tour_length = a.total_tour_length;
	    copy.usedVehicles = a.usedVehicles;
		for (int indexObj = 0; indexObj < 2; indexObj++) {
			copy.costObjectives[indexObj] = a.costObjectives[indexObj];
    	}
		for (int i = 0; i < a.usedVehicles; i++) {
			copy.tour_lengths.add(i, a.tour_lengths.get(i));
			int size = a.tours.get(i).size();
			copy.tours.add(i, new ArrayList<Integer>(size));
			for (int j = 0; j < size; j++) {
				int elem = a.tours.get(i).get(j);
				copy.tours.get(i).add(elem);
			}		
		}
    	
    	return copy;
    }
    
    //get the list with the unvisited customers
    static ArrayList unroutedCustomers(Ant a, VRPTW vrp) {
    	ArrayList<Integer> l = new ArrayList<Integer>(a.toVisit);
    	ArrayList<Integer> idKnownRequests = vrp.getIdAvailableRequests();
    	int count = 0;
    	
    	//collect nodes missing from the ant's solution; depot is considered to be visisted by default
    	for (int city : idKnownRequests) {
    		if (a.visited[city] == false) {
    			l.add(city);
    			count++;
	    		if (count == a.toVisit) {
	    			break;
	    		}
    		}		
    	}
    	
    	return l;
    }

    //choose for an ant as the next city the one with maximal value of heuristic information times pheromone
    static int[] choose_best_next(Ant a, VRPTW vrp)
    {
		int current_city, next_city, salesman = 0, indexTour, startIndex, startIndexTour = 0;
		double value_best = -1.;  /* values in total matrix are always >= 0.0 */
		double help;
		int[] values = new int[2];
		double distance, distanceDepot, arrivalTime, arrivalTimeDepot, beginService, beginServiceDepot;
	    double currentTime = 0, timeDiference = 0,  waitingTIme, waiting, deliveryUrgency, bestBeginService = 0;
	    ArrayList<Request> reqList = vrp.getRequests();
	    ArrayList<Integer> idKnownRequests = vrp.getIdAvailableRequests();
	    ArrayList<Integer> lastCommitedIndexes;
	    int pos;
	    boolean appliedInsertion = false;
	    
		next_city = VRPTW.n;
		
		if (a.addedEmptyTour) {
			startIndex = a.usedVehicles - 1;
		}
		else {
			startIndex = 0;
		}
		
		for (int indexSalesman = startIndex; indexSalesman < a.usedVehicles; indexSalesman++) {
			int lastPos = a.tours.get(indexSalesman).size() - 1;
			current_city = a.tours.get(indexSalesman).get(lastPos);
	    	current_city++;
	    	
	    	for (int city : idKnownRequests) {
			    if (a.visited[city])
				; /* city already visited, do nothing */
			    else {
			    	distance = VRPTW.instance.distance[current_city][city + 1];
			    	arrivalTime = a.currentTime.get(indexSalesman) + reqList.get(current_city).getServiceTime() + distance; 
			    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
			    	
			    	distanceDepot = VRPTW.instance.distance[city + 1][0];
			    	arrivalTimeDepot = beginService + reqList.get(city + 1).getServiceTime() + distanceDepot;
			    	beginServiceDepot = Math.max(arrivalTimeDepot, reqList.get(0).getStartWindow());
			    	
			    	if (VRPTW_ACS.isFeasible(vrp, a, city, beginService, beginServiceDepot, indexSalesman)) {
			    		waiting = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime() - distance;
			    		waitingTIme = Math.max(0.0, waiting);
				    	/*if (waitingTIme == 0) {
				    		waitingTIme = 0.05;
				    	}*/
				    	currentTime = a.currentTime.get(indexSalesman);
				    	if (currentTime == 0) {
				    		currentTime = 0.001;
				    	}
				    	deliveryUrgency = reqList.get(city + 1).getEndWindow() - (a.beginService[current_city] + reqList.get(current_city).getServiceTime() + distance);
				    	/*if (deliveryUrgency == 0) {
				    		deliveryUrgency = 0.05;
				    	}*/
				    	timeDiference = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime();
				    	
						//help = HEURISTIC(current_city, city + 1) * (1.0 / currentTime) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
						//help = HEURISTIC(current_city, city + 1) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
				    	
				    	/*if (a.currentTime.get(indexSalesman) == 0 && reqList.get(city + 1).getStartWindow() == 0) {
				    		help = 1.0 / (weight1 * distance + weight2 * timeDiference);
				    	}
				    	else {
				    		help = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
				    	}*/
						help = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
						
						help = Math.pow(help, beta);
						help = help * Math.pow(pheromone[current_city][city + 1], alpha);
						if (help > value_best) {
						    next_city = city;
						    value_best = help;
						    salesman = indexSalesman;
						    bestBeginService = beginService;
						}
			    	}
					
			    }
			}
		}
		
		//it means that not all the cities are covered and some customers has not yet been serviced
		//by using an insertion heuristic try to insert in the infeasible solution the customers not visited yet
	    //when no more customer with feasible insertions can be found, start a new route/tour and add 
		//one more vehicle for constructing a feasible solution
		if (next_city == VRPTW.n) {
		    //System.out.println("Iter=" + InOut.iteration + ": Before insertion heuristic >> cities to be visited: " + a.toVisit);
		    
			if ((a.toVisit > 0) && (a.toVisit <= 10)) {
				//determine nodes that are not visited yet in the current ant's solution
				ArrayList<Integer> unroutedList = unroutedCustomers(a, vrp);	
				if (appliedInsertion) {
					startIndexTour = a.usedVehicles - 1;
				}
				else {
					startIndexTour = 0;
				}
				lastCommitedIndexes = new  ArrayList<Integer>();
				for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
					 pos = Controller.getLastCommitedPos(index);
					 lastCommitedIndexes.add(pos);
				}
				//skip over committed (defined) nodes when performing insertion heuristic
				InsertionHeuristic.insertUnroutedCustomers(a, vrp, unroutedList, startIndexTour, lastCommitedIndexes);
				appliedInsertion = true;
				//System.out.println("Iter=" + InOut.iteration + ": After insertion heuristic >> cities to be visited: " + a.toVisit);
			}
		    //if no more unrouted customers can be feasible inserted in the solution and there are still 
		    //remaining unrouted customers, add a new tour 
		    if (a.toVisit > 0) {
		    	a.usedVehicles++;
				indexTour = a.usedVehicles - 1;
				a.tours.add(indexTour, new ArrayList<Integer>());
				a.tours.get(indexTour).add(-1); 
				a.tour_lengths.add(indexTour, 0.0);
				a.currentQuantity.add(indexTour, 0.0);
			    a.currentTime.add(indexTour, 0.0);
			    
			    a.addedEmptyTour = true;
			    
			    values[0] = -1;
				values[1] = indexTour; 
		    }
					
		}
		else {
			a.tours.get(salesman).add(next_city);
			a.visited[next_city] = true;
			a.toVisit--;
			a.currentTime.set(salesman, bestBeginService);
			a.beginService[next_city + 1] = bestBeginService;
			double newQuantity = a.currentQuantity.get(salesman) + reqList.get(next_city + 1).getDemand();
			a.currentQuantity.set(salesman, newQuantity);
			
			values[0] = next_city;
			values[1] = salesman;
		}
		
		return values;
    }

    //chooses for an ant as the next city the one with maximal value of heuristic information times pheromone
    static int[] neighbour_choose_best_next(Ant a, VRPTW vrp) {
		int i, current_city, next_city, help_city, salesman = 0, startPos;
		double value_best = -1;  //values in total matrix are always >= 0.0 
		double help;
		double distance, distanceDepot, arrivalTime, arrivalTimeDepot, beginService, beginServiceDepot;
	    double currentTime = 0, timeDiference = 0, waitingTIme, waiting, deliveryUrgency, bestBeginService = 0;
		int[] values = new int[2];
		ArrayList<Request> reqList = vrp.getRequests();
	
		next_city = VRPTW.n;   //next_city = Integer.MAX_VALUE;
		if (a.addedEmptyTour) {
			startPos = a.usedVehicles - 1;
		}
		else {
			startPos = 0;
		}
		
		for (int indexSalesman = startPos; indexSalesman < a.usedVehicles; indexSalesman++) {
			int lastPos = a.tours.get(indexSalesman).size() - 1;
			current_city = a.tours.get(indexSalesman).get(lastPos);
	        current_city++;
	 
			for (i = 0; i < nn_ants; i++) {
			    help_city = VRPTW.instance.nn_list[current_city][i];
			    if ((vrp.getIdAvailableRequests().contains(help_city - 1)) && (!a.visited[help_city - 1])) {
			    	distance = VRPTW.instance.distance[current_city][help_city];
			    	arrivalTime = a.currentTime.get(indexSalesman) + reqList.get(current_city).getServiceTime() + distance; 
			    	beginService = Math.max(arrivalTime, reqList.get(help_city).getStartWindow());
			    	
			    	distanceDepot = VRPTW.instance.distance[help_city][0];
			    	arrivalTimeDepot = beginService + reqList.get(help_city).getServiceTime() + distanceDepot;
			    	beginServiceDepot = Math.max(arrivalTimeDepot, reqList.get(0).getStartWindow());
			    	
			    	if (VRPTW_ACS.isFeasible(vrp, a, help_city - 1, beginService, beginServiceDepot, indexSalesman)) {
			    		waiting = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime() - distance;
			    		waitingTIme = Math.max(0.0, waiting);
			    		/*if (waiting <= 0) {
			    			waitingTIme = 0;
			    		}
			    		else {
			    			waitingTIme = waiting;
			    		}*/
				    	/*if (waitingTIme == 0) {
				    		waitingTIme = 0.05;
				    	}*/
				    	currentTime = a.currentTime.get(indexSalesman);
				    	if (currentTime == 0) {
				    		currentTime = 0.001;
				    	}
				    	deliveryUrgency = reqList.get(help_city).getEndWindow() - (a.beginService[current_city] + reqList.get(current_city).getServiceTime() + distance);
				    	/*if (deliveryUrgency == 0) {
				    		deliveryUrgency = 0.05;
				    	}*/
				    	timeDiference = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime();
				    	
						//help = HEURISTIC(current_city, help_city) * (1.0 / currentTime) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
					    //help = HEURISTIC(current_city, help_city) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
				    	
				    	/*if (a.currentTime.get(indexSalesman) == 0 && reqList.get(help_city).getStartWindow() == 0) {
				    		help = 1.0 / (weight1 * distance + weight2 * timeDiference);
				    	}
				    	else {
				    		help = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
				    	}*/
				    	help = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
					    
						help = Math.pow(help, beta);
						help = help * Math.pow(pheromone[current_city][help_city], alpha);
						if (help > value_best) {
						    value_best = help;
						    next_city = help_city - 1;
						    salesman = indexSalesman;
						    bestBeginService = beginService;
						}
			    	}
			    	
			    }
			}
		}
			
		if (next_city == VRPTW.n) {
		    // all cities in nearest neighbor list were already visited 
		    values = choose_best_next(a, vrp);
		    return values;
		}
		else {
			a.tours.get(salesman).add(next_city);
		    a.visited[next_city] = true;
		    a.toVisit--;
		    a.currentTime.set(salesman, bestBeginService);
			a.beginService[next_city + 1] = bestBeginService;
			double newQuantity = a.currentQuantity.get(salesman) + reqList.get(next_city + 1).getDemand();
			a.currentQuantity.set(salesman, newQuantity);
		    values[0] = next_city;
			values[1] = salesman;
			return values;
		}
		
    }
    
    static void choose_closest_nn(Ant a, int indexSalesman, VRPTW vrp)
    {
		int current_city, next_city, indexTour;
	    double distance, distanceDepot, arrivalTime, arrivalTimeDepot, beginService, beginServiceDepot;
	    double timeDiference, deliveryUrgency, bestBeginService = 0, minValue, metricValue;
	    ArrayList<Request> reqList = vrp.getRequests();
	    ArrayList<Integer> idKnownRequests = vrp.getIdAvailableRequests();
	    
	    while (a.toVisit > 0) {
	    	next_city = VRPTW.n;
			int lastPos = a.tours.get(indexSalesman).size() - 1;
			current_city = a.tours.get(indexSalesman).get(lastPos);
			current_city++;
			
			minValue = Integer.MAX_VALUE;  
			for (int city : idKnownRequests) {
			    if (a.visited[city])
				;  //city already visited 
			    else {
			    	distance = VRPTW.instance.distance[current_city][city + 1];
			    	arrivalTime = a.currentTime.get(indexSalesman) + reqList.get(current_city).getServiceTime() + distance; 
			    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
			    	
			    	distanceDepot = VRPTW.instance.distance[city + 1][0];
			    	arrivalTimeDepot = beginService + reqList.get(city + 1).getServiceTime() + distanceDepot;
			    	beginServiceDepot = Math.max(arrivalTimeDepot, reqList.get(0).getStartWindow());
			    	
			    	if (VRPTW_ACS.isFeasible(vrp, a, city, beginService, beginServiceDepot, indexSalesman)) {
			    		//compute the value of the "closeness" metric; this metric tries to account
			    		//for both geographical and temporal closeness of customers
			    		timeDiference = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime();
			    		deliveryUrgency = reqList.get(city + 1).getEndWindow() - (a.beginService[current_city] + reqList.get(current_city).getServiceTime() + distance);
			    		
			    		/*if (a.currentTime.get(indexSalesman) == 0 && reqList.get(city + 1).getStartWindow() == 0) {
			    			metricValue = weight1 * distance + weight2 * timeDiference;
				    	}
				    	else {
				    		metricValue = weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency;
				    	}*/
			    		//metricValue = 0.5 * distance + 0.3 * timeDiference + 0.2 * deliveryUrgency;
			    		//metricValue = initialWeight1 * distance + initialWeight2 * timeDiference + initialWeight3 * deliveryUrgency;
			    		metricValue = weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency;
			    		
						if (metricValue < minValue) {
						    next_city = city;
						    minValue = metricValue;
						    bestBeginService = beginService;
						}
			    	}
			    }
			}
			
			//no more nodes can be feasible added in the tour
			if (next_city == VRPTW.n) {
				break;
			}
			else {
				a.tours.get(indexSalesman).add(next_city);
				a.visited[next_city] = true; 
				a.toVisit--;
				a.currentTime.set(indexSalesman, bestBeginService);
				a.beginService[next_city + 1] = bestBeginService;
				a.currentQuantity.set(indexSalesman, a.currentQuantity.get(indexSalesman) + reqList.get(next_city + 1).getDemand());
			}
	    }
		
		
    }  
   
   static void choose_closest_next(Ant a, int indexSalesman, VRPTW vrp)
    {
		int current_city, next_city, indexTour;
	    double distance, distanceDepot, arrivalTime, arrivalTimeDepot, beginService, beginServiceDepot;
	    double timeDiference, deliveryUrgency, bestBeginService = 0, minValue, metricValue;
	    ArrayList<Request> reqList = vrp.getRequests();
	    ArrayList<Integer> idKnownRequests = vrp.getIdAvailableRequests();
	    
		next_city = VRPTW.n;
		int lastPos = a.tours.get(indexSalesman).size() - 1;
		current_city = a.tours.get(indexSalesman).get(lastPos);
		current_city++;
		
		minValue = Integer.MAX_VALUE;  
		for (int city : idKnownRequests) {
		    if (a.visited[city])
			;  //city already visited 
		    else {
		    	distance = VRPTW.instance.distance[current_city][city + 1];
		    	arrivalTime = a.currentTime.get(indexSalesman) + reqList.get(current_city).getServiceTime() + distance; 
		    	beginService = Math.max(arrivalTime, reqList.get(city + 1).getStartWindow());
		    	
		    	distanceDepot = VRPTW.instance.distance[city + 1][0];
		    	arrivalTimeDepot = beginService + reqList.get(city + 1).getServiceTime() + distanceDepot;
		    	beginServiceDepot = Math.max(arrivalTimeDepot, reqList.get(0).getStartWindow());
		    	
		    	if (VRPTW_ACS.isFeasible(vrp, a, city, beginService, beginServiceDepot, indexSalesman)) {
		    		//compute the value of the "closeness" metric; this metric tries to account
		    		//for both geographical and temporal closeness of customers
		    		timeDiference = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime();
		    		deliveryUrgency = reqList.get(city + 1).getEndWindow() - (a.beginService[current_city] + reqList.get(current_city).getServiceTime() + distance);
		    		
		    		/*if (a.currentTime.get(indexSalesman) == 0 && reqList.get(city + 1).getStartWindow() == 0) {
		    			metricValue = weight1 * distance + weight2 * timeDiference;
			    	}
			    	else {
			    		metricValue = weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency;
			    	}*/
		    		//metricValue = 0.5 * distance + 0.3 * timeDiference + 0.2 * deliveryUrgency;
		    		//metricValue = initialWeight1 * distance + initialWeight2 * timeDiference + initialWeight3 * deliveryUrgency;
		    		metricValue = weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency;
		    		
					if (metricValue < minValue) {
					    next_city = city;
					    minValue = metricValue;
					    bestBeginService = beginService;
					}
		    	}
		    }
		}
		//it means that not all the cities are covered and some customers has not yet been serviced
		//by using insertion heuristic try to insert in the current tour of the solution under construction
		//the customers not visited yet
	    //when no more customer with feasible insertions can be found, start a new route/tour and add 
		//one more vehicle for constructing a feasible solution
		if (next_city == VRPTW.n) {
			//System.out.println("Cities to be visited: " + a.toVisit);
			
			//determine nodes that are not visited yet in the current tour
		    /*ArrayList<Integer> unroutedList = unroutedCustomers(a, vrp);	    
		    InsertionHeuristic.insertUnroutedCustomers(a, vrp, unroutedList, a.usedVehicles - 1, 1);*/
		    
		    //if no more unrouted customers can be feasible inserted in the solution and there are still 
		    //remaining unrouted customers, add a new tour 
		    if (a.toVisit > 0) {
		    	a.usedVehicles++;
		    	indexTour = a.usedVehicles - 1;
				a.tours.add(indexTour, new ArrayList<Integer>());
				a.tours.get(indexTour).add(-1); 
				a.tour_lengths.add(indexTour, 0.0);
				a.currentQuantity.add(indexTour, 0.0);
			    a.currentTime.add(indexTour, 0.0);
		    }
			
		}
		else {
			a.tours.get(indexSalesman).add(next_city);
			a.visited[next_city] = true; 
			a.toVisit--;
			a.currentTime.set(indexSalesman, bestBeginService);
			a.beginService[next_city + 1] = bestBeginService;
			a.currentQuantity.set(indexSalesman, a.currentQuantity.get(indexSalesman) + reqList.get(next_city + 1).getDemand());
		}
		
    } 

    //Choose for an ant probabilistically a next city among all unvisited cities in the current city's candidate list
    static int[] neighbour_choose_and_move_to_next(Ant a, VRPTW vrp) {
		int i, j, help, city, salesman = 0;
		int current_city = 0;
		double rnd, partial_sum = 0., sum_prob = 0.0;
		double prob_ptr[][];
		double help1;
		int[] values = new int[2];
		int[] tempCities = new int[a.usedVehicles]; 
		double distance, distanceDepot, arrivalTime, currentTime, arrivalTimeDepot, beginService, beginServiceDepot;
	    double waitingTIme, waiting, deliveryUrgency, timeDiference = 0;
	    ArrayList<Request> reqList = vrp.getRequests();
	
		
		if ((q_0 > 0.0) && (Utilities.random01() < q_0)) {
		    /*
		     * with a probability q_0 make the best possible choice
		     * according to pheromone trails and heuristic information, this corresponds to exploitation
		     */
		    /*
		     * we first check whether q_0 > 0.0, to avoid the very common case
		     * of q_0 = 0.0 to have to compute a random number, which is
		     * expensive computationally
		     */
		    values = neighbour_choose_best_next(a, vrp);
		    //values = choose_best_next(a, vrp);
		    return values; 
		}
	
		//prob_ptr = prob_of_selection;
		prob_ptr = new double[a.usedVehicles][nn_ants + 1];
		for (j = 0; j < a.usedVehicles; j++) {
			for (i = 0; i < nn_ants; i++) {
				prob_ptr[j][i] = Double.POSITIVE_INFINITY;
			}
		}
		for (j = 0; j < (a.usedVehicles - 1); j++) {
			prob_ptr[j][nn_ants] = 0;
		}
		prob_ptr[a.usedVehicles - 1][nn_ants] = Double.POSITIVE_INFINITY;
	
		for (int indexSalesman = 0; indexSalesman < a.usedVehicles; indexSalesman++) {
			/* current_city city of ant k */
			int lastPos = a.tours.get(indexSalesman).size() - 1;
			current_city = a.tours.get(indexSalesman).get(lastPos);
	    	current_city++;
	    
			for (i = 0; i < nn_ants; i++) {
				city = VRPTW.instance.nn_list[current_city][i];
				distance = VRPTW.instance.distance[current_city][city];
		    	arrivalTime = a.currentTime.get(indexSalesman) + reqList.get(current_city).getServiceTime() + distance; 
		    	beginService = Math.max(arrivalTime, reqList.get(city).getStartWindow());
		    	
		    	distanceDepot = VRPTW.instance.distance[city][0];
		    	arrivalTimeDepot = beginService + reqList.get(city).getServiceTime() + distanceDepot;
		    	beginServiceDepot = Math.max(arrivalTimeDepot, reqList.get(0).getStartWindow());
				
			    if (!(vrp.getIdAvailableRequests().contains(city - 1)) || (a.visited[city - 1]) || (!VRPTW_ACS.isFeasible(vrp, a, city - 1, beginService, beginServiceDepot, indexSalesman)))
			    	prob_ptr[indexSalesman][i] = 0.0; /* city already visited */
			    else if ((VRPTW_ACS.isFeasible(vrp, a, city - 1, beginService, beginServiceDepot, indexSalesman)) && (vrp.getIdAvailableRequests().contains(city - 1)) && !(a.visited[city - 1])) {    	
		    		waiting = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime() - distance;
		    		waitingTIme = Math.max(0.0, waiting);
			    	/*if (waitingTIme == 0) {
			    		waitingTIme = 0.05;
			    	}*/
			    	deliveryUrgency = reqList.get(city).getEndWindow() - (a.beginService[current_city] + reqList.get(current_city).getServiceTime() + distance);
			    	/*if (deliveryUrgency == 0) {
			    		deliveryUrgency = 0.05;
			    	}*/
			    	currentTime = a.currentTime.get(indexSalesman);
			    	if (currentTime == 0) {
			    		currentTime = 0.001;
			    	}
			    	timeDiference = beginService - a.beginService[current_city] - reqList.get(current_city).getServiceTime();
			    	
					//help1 = HEURISTIC(current_city, city) *  (1.0 / currentTime) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
			    	//help1 = HEURISTIC(current_city, city) * (1.0 / waitingTIme) * (1.0 / deliveryUrgency);
			    	
			    	/*if (a.currentTime.get(indexSalesman) == 0 && reqList.get(city).getStartWindow() == 0) {
			    		help1 = 1.0 / (weight1 * distance + weight2 * timeDiference);
			    	}
			    	else {
			    		help1 = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
			    	}*/
			    	help1 = 1.0 / (weight1 * distance + weight2 * timeDiference + weight3 * deliveryUrgency);
					help1 = Math.pow(help1, beta);
					help1 = help1 * Math.pow(pheromone[current_city][city], alpha);
					prob_ptr[indexSalesman][i] = help1;
					tempCities[indexSalesman] = current_city; 
					sum_prob += prob_ptr[indexSalesman][i];
		    	}
			  
			    	
		    }
		}
		
		if (sum_prob <= 0.0) {
		    /* All cities from the candidate  are tabu (are already visited) */
			values = choose_best_next(a, vrp);
			return values;
		} else {
		    /*
		     * at least one neighbor is eligible, choose one according to the
		     * selection probabilities
		     */
		    rnd = Utilities.random01();
		    rnd *= sum_prob;
		    i = 0;
		    boolean done = false, forcedEnd = false;
		    partial_sum = 0;
		    for (int indexSalesman = 0; indexSalesman < a.usedVehicles && !done; indexSalesman++) {
		    	i = 0;
		    	partial_sum += prob_ptr[indexSalesman][i];
			    /* This loop always stops because prob_ptr[nn_ants] == HUGE_VAL */
			    while (partial_sum <= rnd) {
					i++;
					if (i < 0) {
						System.out.println("Iter=" + InOut.iteration + " Test: indexSalesman= " + indexSalesman + " i= " + i);
						partial_sum += Double.POSITIVE_INFINITY;
						forcedEnd = true;
						break;
					}
					if (i < prob_ptr[indexSalesman].length) {
						//System.out.println("Test: indexSalesman= " + indexSalesman + " i= " + i);
						partial_sum += prob_ptr[indexSalesman][i];
						salesman = indexSalesman;
					}
					else if (i >= prob_ptr[indexSalesman].length) {
						break;
					}
					//add a big value to the partial_sum to be sure that the while loop ends
					else if (indexSalesman == (a.usedVehicles- 1) && i >= prob_ptr[indexSalesman].length && partial_sum <= rnd) {
						partial_sum += Double.POSITIVE_INFINITY;
						forcedEnd = true;
					}	
			    }
			    if (partial_sum > rnd) {
			    	done = true;
			    	if (!forcedEnd) {
			    		salesman = indexSalesman;
			    	}
			    	else { //choose randomly a salesman to whom add the city
			    		salesman = (int)(Math.random() * a.usedVehicles);
			    	}
			    	
			    }
			    
		    }
		    
		    /*
		     * This may very rarely happen because of rounding if rnd is close to 1.
		     */
		    if (i == nn_ants) {
		    	values = neighbour_choose_best_next(a, vrp);
		    	//values = choose_best_next(a, vrp);
				return values;
		    }
		    //System.out.println("nn_ants=" + nn_ants + " salesman=" + salesman + " i=" + i);
		    current_city = tempCities[salesman];
		    int maxIndex = 0;
		    double maxValue = Double.MIN_VALUE;
		    if (i < 0) {
		    	maxValue = prob_ptr[salesman][0];
		    	for (j = 1; j < nn_ants; j++) { 
		    	  if (prob_ptr[salesman][j] > maxValue) {
		    		  maxValue = prob_ptr[salesman][j];
		    		  maxIndex = j;
		    	  }
		    	}
		    	i = maxIndex;
		    }
		    
		    help = VRPTW.instance.nn_list[current_city][i];
		    
		    distance = VRPTW.instance.distance[current_city][help];
		    arrivalTime = a.currentTime.get(salesman) + reqList.get(current_city).getServiceTime() + distance; 
	    	beginService = Math.max(arrivalTime, reqList.get(help).getStartWindow());
		    
		    a.tours.get(salesman).add(help - 1);
		    a.visited[help - 1] = true;
		    a.toVisit--;
		    a.currentTime.set(salesman, beginService);
			a.beginService[help] = beginService;
			double newQuantity = a.currentQuantity.get(salesman) + reqList.get(help).getDemand();
			a.currentQuantity.set(salesman, newQuantity);
		    
		    values[0] = help - 1;
			values[1] = salesman;
			return values;
		}
		
    }

    //reinforces the edges used in ant's solution as in ACS
    static void global_acs_pheromone_update(Ant a) {
		int i, j, h, k, size;
		double d_tau;
	
		d_tau = 1.0 / (double) a.total_tour_length;
		//d_tau = 1.0 / ((double) a.total_tour_length * (double) a.usedVehicles);
		//d_tau = 1.0 / ((double) a.total_tour_length * (double) a.usedVehicles * 0.1);
		//d_tau = 1.0 / ((double) a.total_tour_length + (double) a.usedVehicles * 0.01);
		/*double weightedSum = 0.5 * (double)a.total_tour_length + 0.5 * (double)a.usedVehicles;
		d_tau = 1.0 / Math.pow(weightedSum, 2.0);*/
	
		for (i = 0; i < a.usedVehicles; i++) {
			size = a.tours.get(i).size();
			for (k = 0; k < size - 1; k++)  {
			    j = a.tours.get(i).get(k);
			    h = a.tours.get(i).get(k + 1);	
			    
			    j++;
			    h++;
		
			    pheromone[j][h] = (1. - rho) * pheromone[j][h] + rho * d_tau;
			    pheromone[h][j] = pheromone[j][h];
		
			    /*total[h][j] = Math.pow(pheromone[h][j], alpha) * Math.pow(HEURISTIC(h, j), beta);
			    total[j][h] = total[h][j];*/
			}
		}
		
    }

    //removes some pheromone on edge just passed by the ant
    static void local_acs_pheromone_update(Ant a, int indexSalesman) {
		int h, j;
		
		int lastPos = a.tours.get(indexSalesman).size() - 1;
		j = a.tours.get(indexSalesman).get(lastPos);
		h = a.tours.get(indexSalesman).get(lastPos - 1);	
		
		j++;
		h++;

		/* still additional parameter has to be introduced */
		pheromone[h][j] = (1. - local_rho) * pheromone[h][j] + local_rho * trail_0;
		pheromone[j][h] = pheromone[h][j];
		/*total[h][j] = Math.pow(pheromone[h][j], alpha) * Math.pow(HEURISTIC(h, j), beta);
		total[j][h] = total[h][j];*/
    }

    //copy solution from ant a1 into ant a2
    static void copy_from_to(Ant a1, Ant a2, VRPTW instance) {
		int i, j;
	
		Ants.ant_empty_memory(a2, instance);
		
		a2.total_tour_length = a1.total_tour_length;
		/*a2.longest_tour_length = a1.longest_tour_length;
		a2.indexLongestTour = a1.indexLongestTour;*/
		a2.toVisit = a1.toVisit;
	
		for (int indexObj = 0; indexObj < 2; indexObj++) {
		   a2.costObjectives[indexObj] = a1.costObjectives[indexObj];
    	}
		
		if (a2.usedVehicles < a1.usedVehicles) {
			for (int index = a2.usedVehicles; index < a1.usedVehicles; index++) {
				a2.tour_lengths.add(index, 0.0);
				a2.tours.add(index, new ArrayList<Integer>());
				a2.currentQuantity.add(index, 0.0);
				a2.currentTime.add(index, 0.0);		
				/*a2.earliestTime.add(index, new ArrayList<Double>());
				a2.latestTime.add(index, new ArrayList<Double>());*/
			}
		}
		
		for (i = 0; i < a1.usedVehicles; i++) {
			a2.tour_lengths.set(i, a1.tour_lengths.get(i));
			a2.currentQuantity.set(i, a1.currentQuantity.get(i));
			a2.currentTime.set(i, a1.currentTime.get(i));
			int size = a1.tours.get(i).size();
			for (j = 0; j < size; j++) {
				int elem = a1.tours.get(i).get(j);
				a2.tours.get(i).add(elem);
			}
			
		}
		for (i = 0; i < VRPTW.n; i++) {
			a2.visited[i] = a1.visited[i];
		}
		a2.usedVehicles = a1.usedVehicles;
		
		for (i = 0; i < (VRPTW.n + 1); i++) {
			a2.beginService[i] = a1.beginService[i];
		}
    }
    
    static double computeToursAmplitude(Ant a) {
    	double min, max;
		int i;

		min = a.tour_lengths.get(0);
		max = a.tour_lengths.get(0);
		for (i = 1; i < a.tours.size(); i++) {
		    if (a.tour_lengths.get(i) < min) {
				min = a.tour_lengths.get(i);
		    }
		    if (a.tour_lengths.get(i) > max) {
				max = a.tour_lengths.get(i);
		    }	    
		}
		
		return (max - min);
    }

    //generate a nearest neighbor tour and compute tour length using only the available nodes (nodes known so far)
    static double nn_tour(VRPTW instance) {
    	int step, salesman = 0;
    	double sum = 0, sum1 = 0, scalledValue = 0, noVehicles = 1.0;
    	
    	ant_empty_memory(ants[0], instance);
    	step = 0;
    	
    	for (int i = 0; i < ants[0].usedVehicles; i++) {
    		//place the ant on the depot city, which is the start city of each tour
			// -1 is a special marker for the deport city, so that it's not be confused with the rest of the cities
			// all the rest of the cities are represented by integer values > 0
    		ants[0].tours.get(i).add(-1);  	
    	}
		
    	//there are still left available (known) cities to be visited
		while (ants[0].toVisit > 0) {  	
			salesman = ants[0].usedVehicles - 1;
			choose_closest_next(ants[0], salesman, instance);	 
		}
		
		//System.out.println("Cities to be visited: " + ants[0].toVisit);
		
		int nrTours = ants[0].usedVehicles;
		for (int i = 0; i < nrTours; i++) {
			step = ants[0].tours.get(i).size();
			ants[0].tours.get(i).add(step, -1);
			ants[0].tour_lengths.set(i, VRPTW.compute_tour_length(ants[0].tours.get(i)));
			sum1 += ants[0].tour_lengths.get(i);
		}
		
		ants[0].total_tour_length = sum1;
		
		if (VRPTW_ACS.ls_flag) {
			//ants[0] = VRPTW_ACS.local_search(ants[0], instance);
			ants[0] = VRPTW_ACS.relocateMultipleRouteIterated(ants[0], instance);
			ants[0] = VRPTW_ACS.exchangeMultipleRouteIterated(ants[0], instance);
			
			//compute new distances and update longest tour
			for (int l = 0; l < ants[0].usedVehicles; l++) {
				ants[0].tour_lengths.set(l, VRPTW.compute_tour_length(ants[0].tours.get(l)));
				sum += ants[0].tour_lengths.get(l);
			}
			ants[0].total_tour_length = sum;
		}
		
		/*for (int j = 0; j < ants[0].usedVehicles; j++) {
			lastCommitted.add(j, 0);  
		}*/
		
		//System.out.println("Initial (nearest neighbour tour) longest tour length: " + longestTourLength);
		double scalingValue = Controller.getScalingValue();
		if (scalingValue != 0) {
			scalledValue = ants[0].total_tour_length / scalingValue;
		}
		System.out.println("\nInitial (nearest neighbour tour) total tour length: " + ants[0].total_tour_length + " (scalled value = " + scalledValue + "); Number of vehicles used: " + ants[0].usedVehicles);
		sum1 = ants[0].total_tour_length;
		noVehicles = ants[0].usedVehicles;
		
		/*for (int i = 0; i < nrTours; i++) {
			int tourLength = ants[0].tours.get(i).size();
			for (int j = 0; j < tourLength; j++) {
				int city = ants[0].tours.get(i).get(j);
				city = city + 1;  //so as to correspond to the city indexes from the VRPTW input file
				System.out.print(city + " ");		
			}
			System.out.println();
		}*/
		
		//initialize best solution so far with this solution constructed by the nearest neighbour heuristic
		Ants.copy_from_to(ants[0], Ants.best_so_far_ant, instance);
		
		/*for (int j = 0; j < Ants.best_so_far_ant.usedVehicles; j++) {
			lastCommitted.add(j, 0);  
		}*/
		
		//sum = ants[0].total_tour_length;
		ant_empty_memory(ants[0], instance);
		
		//return sum;
		//double value = sum1 * noVehicles; 
		//double value = sum1 * ants[0].usedVehicles * 0.1; 
		//double value = sum1 + ants[0].usedVehicles * 0.01; 
		/*double weightedSum = 0.5 * (double)sum1 + 0.5 * (double)ants[0].usedVehicles;
		double value = Math.pow(weightedSum, 2.0);*/
		return sum1;
		//return value;
    }


}
