package aco;

import java.util.ArrayList;

import aco.Ants.Ant;

/*
 * A class containing utilities methods for constructing the Pareto front that contains
 * non-dominated solutions found during the execution of the ACO algorithm
 * These solutions are stored in an external set and are returned to the output when the
 * execution of the algorithm finishes
 */

public class ParetoFront {

	//an external set containing the non-dominated solutions found during the execution of the algorithm
	//these solutions represent the output of the P-ACO algorithm 
	static ArrayList<Ant> bestSoFarPareto = new ArrayList<Ant>();
	
	//an external set containing the non-dominated solutions found during the current iteration
	//the solutions from this set will be used to update the bestSoFarPareto set after all the solutions
	//from an iteration are computed
	static ArrayList<Ant> iterationPareto = new ArrayList<Ant>();
	
	
	 //return 0 if a1 == a2; return 1 if a1 dominates a2; return -1 if a1 NOT dominates a2  
	 
	/*public static int solutionDominates(Ant a1, Ant a2) {
		for (int indexObj = 0; indexObj < TSP_ACO.k; indexObj++) {
    		//if any objective is worse, a1 can't dominate a2
			if (a1.costObjectives[indexObj] > a2.costObjectives[indexObj]) return -1;
	
		    //if any objective is better, then a1 dominates a2
			if (a1.costObjectives[indexObj] < a2.costObjectives[indexObj]) return 1;	    
    	}
		//otherwise a1 == a2
		return 0;
	}*/
	
	public static byte solutionDominates(Ant solution1, Ant solution2) {
		boolean first = true;
		int l = 0;
		int nObjectives = 2;
		
		while (l < nObjectives && solution1.costObjectives[l] == solution2.costObjectives[l])
			l++;
		if (l == nObjectives)
			return -2;
			
		if (solution1.costObjectives[l] > solution2.costObjectives[l]) {
			first = false;
		}

		boolean dominates = true; //first dominates the second
		int obj = 1;
		
		if (first) {
			while (dominates && obj < nObjectives && solution1.costObjectives[obj] <= solution2.costObjectives[obj])
				obj++;
		}
		else {
			while (dominates && obj < nObjectives && solution2.costObjectives[obj] <= solution1.costObjectives[obj])
				obj++;
		}
				
		if (obj != nObjectives)
			return 0; //no dominance
		else if (first)
			return -1; //the first solution dominates the second
		else
			return 1; //the second solution dominates
	}
	
	//update the iteration Pareto with this solution obtained in the current iteration
	public static void paretoUpdateWithSolution(ArrayList<Ant> iterPareto, Ant a) {
		int i = 0;
		boolean dominated = false;
		
		while (i < iterPareto.size() && !dominated) {
			/* If exist one solution in the pareto equal to this solution, then we can say this 
			 * solution is already included in the pareto, therefore there is no solution in the 
			 * pareto that can dominate this one, and there is no solution in the pareto dominated 
			 * by this one. And, if there is one solution in the pareto that dominates this solution, 
			 * then this solution can't dominate any solution in the pareto.
	        */
			if (solutionDominates(iterPareto.get(i), a) < 0) {
				dominated = true;
	        }
			
			/* If this solution dominates one solution in the pareto, then there is not any solution 
			 * in the pareto that dominates this one.
		    */
			else if (solutionDominates(a, iterPareto.get(i)) == -1) {
				//remove the dominated one
				iterPareto.remove(i);
				
				//check for other dominated solutions in the Pareto
				while (i < iterPareto.size()) {
	                if (solutionDominates(a, iterPareto.get(i)) == -1) {
	                	//remove the dominated one
	                	iterPareto.remove(i);
	                }
	                else { i++; }
	            }
				//add this solution to the pareto, so it becomes dominated 
				iterPareto.add(a);
				dominated = true;
			}
			//look at next solution in the Pareto
	        else { i++; }		
		}
		
		//if this solution it's not dominated, then it should be included
	    if (!dominated) {
	    	iterPareto.add(a);
	    }
	}
	
	//update the best so far pareto set with the non-dominated solutions from the iteration Pareto set
	public static void paretoUpdate(ArrayList<Ant> bestSoFarPareto, ArrayList<Ant> iterPareto) {
		int i,j;
		boolean dominated;
	    int nrSolutions = iterPareto.size();
	    Ant solution;
	    
	    for (i = 0; i < nrSolutions; i++) {
	    	solution = iterPareto.get(i);
	    	
	    	j = 0;
	        dominated = false;
	        
	        while (j < bestSoFarPareto.size() && !dominated) {
	        	 /* If exist one solution in the pareto equal to this solution, then we can say this 
	        	  * solution is already included in the pareto, therefore there is no solution in the 
	        	  * pareto that can dominate this one, and there is no solution in the pareto dominated 
	        	  * by this one. And, if there is one solution in the pareto that dominates this
	                solution, then this solution can't dominate any solution in the pareto. */
	        	if (solutionDominates(bestSoFarPareto.get(j), solution) < 0) {
	                dominated = true;
	            }
	        	/* If this solution dominates one solution in the pareto, then there is not any 
	        	 * solution in the pareto that dominates this one. */
	        	else if (solutionDominates(solution, bestSoFarPareto.get(j)) == -1) {
	        		//remove the dominated one
	        		bestSoFarPareto.remove(j);
	        		
	        		//check for other dominated solutions in the Pareto
		        	while (j < bestSoFarPareto.size()) {
		        		if (solutionDominates(solution, bestSoFarPareto.get(j)) == -1) {
		        			//remove the dominated one
		        			bestSoFarPareto.remove(j);
		        		}
		        		else { j++; }
		        	}
		        	//add this solution to the pareto 
		        	bestSoFarPareto.add(solution);
					dominated = true;
		        }
	        	//look at next solution in the Pareto
		        else { j++; }
	        	
	        }
	        //if this solution it's not included and it's not dominated, then should be included  
	        if (!dominated) {
	        	bestSoFarPareto.add(solution);
	        }
	    	
	    }
	
	}
	
	public static ArrayList<Ant> getBestSoFarPareto() {
		return bestSoFarPareto;
	}

	public static void setBestSoFarPareto(ArrayList<Ant> bestSoFarPareto) {
		ParetoFront.bestSoFarPareto = bestSoFarPareto;
	}

	public static ArrayList<Ant> getIterationPareto() {
		return iterationPareto;
	}

	public static void setIterationPareto(ArrayList<Ant> iterationPareto) {
		ParetoFront.iterationPareto = iterationPareto;
	}
	
	
}
