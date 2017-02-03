package aco;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/** the controller is the central part of the algorithm, which reads the benchmark data,
 * initializes the data structures, builds an initial solution using nearest neighbor heuristic 
 * and starts the ant colony, once the working day has started
 */
public class Controller {

	//length of a working day in seconds
	private static int workingDay = 100;   
	
	//number of time slices
	private static int noTimeSlices = 50;  
	
	//file name to be used for input data set
	public static String vrpInstance = "r103";   //r104
	
	//dynamic level, which gives the proportion of the dynamic requests (available time > 0) from the DVRPTW instance
	private static double dynamicLevel = 0.1;  //0.0  //0.1  //0.5  //1.0
	
	private static double scalingValue;
	
	private static int idLastAvailableNode = 0;
	
	public static int addedNodes = 0;
	
	
	public static double getScalingValue() {
		return scalingValue;
	}

	public static void setScalingValue(double scalingValue) {
		Controller.scalingValue = scalingValue;
	}

	public static int getIdLastAvailableNode() {
		return idLastAvailableNode;
	}

	public static void setIdLastAvailableNode(int idLastAvailableNode) {
		Controller.idLastAvailableNode = idLastAvailableNode;
	}

	//get a list of new available (known) nodes at the given time moment  
    public static ArrayList<Integer> countNoAvailableNodes(ArrayList<Request> dynamicRequests, double time) {
    	int i, id;
    	int pos = Controller.getIdLastAvailableNode();
    	ArrayList<Integer> nodesList = new ArrayList<Integer>();
    	
    	for (i = pos; i < dynamicRequests.size(); i++) {
    		if (time >= dynamicRequests.get(i).getAvailableTime()) {
    			id = dynamicRequests.get(i).getId() - 1;
    			nodesList.add(id);
    		}
    		else {
    			break;
    		}
    	}
    	Controller.setIdLastAvailableNode(i);
    	
    	return nodesList;
    }
    
    //get the position of the last committed node in the tour designated by indexTour from the best so far ant
    public static int getLastCommitedPos(int indexTour) {
    	int pos = 0;
    	int node;
    	int tourLength;
    	
    	if (indexTour < Ants.best_so_far_ant.usedVehicles) {
    		tourLength = Ants.best_so_far_ant.tours.get(indexTour).size();
	    	for (int i = 1; i < tourLength - 1; i++) {
	    		node = Ants.best_so_far_ant.tours.get(indexTour).get(i);
	    		if (Ants.committedNodes[node]) {
	    			pos++;
	    		}
	    		else {
	    			break;
	    		}
	    	}
    	}
    	
    	return pos;
    }
    
    //check if there are any nodes in the tours of the best so far solution that should be marked as committed
    public static boolean checkNewCommittedNodes(Ants.Ant bestAnt, VRPTW instance, int indexTimeSlice, double lengthTimeSlice) {
    	boolean result = false;
    	int indexTour = 0;
    	int tourLength = 0;
    	int node = 0, startPos = 0, count = 0;
    	
    	while (indexTour < bestAnt.usedVehicles) {
    		if (count >= 50) {
    			System.out.println("Index tour=" + indexTour + ", used vehicles=" + bestAnt.usedVehicles + ", tour length=" + tourLength);
    		}
    		
    		//skip for already committed nodes
    		tourLength = bestAnt.tours.get(indexTour).size();
    		startPos = getLastCommitedPos(indexTour);
    		for (int i = startPos + 1; i < tourLength - 1; i++) {
    			node = bestAnt.tours.get(indexTour).get(i);
    			//check condition for a node to be committed
    			if (bestAnt.beginService[node + 1] <= indexTimeSlice * lengthTimeSlice) {
    				if (!Ants.committedNodes[node]) {
    					return true;
    				}
    				else {
    					continue;
    				}
    			}
    			else {
    				indexTour++;
    				break;
    			}
    		}
    		//if all the nodes from this tour are committed, move to the next tour for checking if it
    		//contains nodes that must be committed
    		if (indexTour < bestAnt.usedVehicles) {
    			startPos = getLastCommitedPos(indexTour);
    			tourLength = bestAnt.tours.get(indexTour).size();
    			if (startPos == tourLength - 2) {
	    			indexTour++;
	    		}
    		}
    		count++;
    		
    	}
    	
    	return result;
    }
    
    //commit nodes from the tours of the best so far solution, that will have their position fixed when
    //they will be copied in the ants'solutions
    // block part of the best solution that is being/has been visited
    public static void commitNodes(Ants.Ant bestAnt, VRPTW instance, int indexTimeSlice, double lengthTimeSlice) {
    	int indexTour = 0;
    	int tourLength = 0;
    	int node = 0, startPos = 0, count = 0;
    	
    	while (indexTour < bestAnt.usedVehicles) {
    		//skip for already committed nodes
    		tourLength = bestAnt.tours.get(indexTour).size();
    		/*if (indexTimeSlice >= 30) {
    			System.out.println("Index tour=" + indexTour + ", used vehicles=" + bestAnt.usedVehicles + ", last committed=" + Ants.lastCommitted.get(indexTour) + ", tour length=" + tourLength);
    		}*/
    		if (count >= 50) {
    			System.out.println("Index tour=" + indexTour + ", used vehicles=" + bestAnt.usedVehicles + ", tour length=" + tourLength);
    		}
    		startPos = getLastCommitedPos(indexTour);
    		for (int i = startPos + 1; i < tourLength - 1; i++) {
    			node = bestAnt.tours.get(indexTour).get(i);
    			//check condition for a node to be committed
    			if ((bestAnt.beginService[node + 1] <= indexTimeSlice * lengthTimeSlice) &&
    			    (!Ants.committedNodes[node])) {
    					Ants.committedNodes[node] = true;
    					//Ants.lastCommitted.set(indexTour, i);
    			}
    			else {
    				indexTour++;
    				break;
    			}
    		}
    		//if all the nodes from this tour were committed (the depot from the start and 
			//end of a tour are assumed to be committed by default), move to the next tour
    		if (indexTour < bestAnt.usedVehicles) {
    			startPos = getLastCommitedPos(indexTour);
    			tourLength = bestAnt.tours.get(indexTour).size();
	    		if (startPos == tourLength - 2) {
	    			indexTour++;
	    		}
    		}
    		
    		count++;
			
    	}
    	
    }
	
	public static void main(String[] args) {
       long startTime, endTime;
       double currentTime, scalingValue, newStartWindow, newEndWindow, newServiceTime, newAvailableTime;
       //counter which stores the number of the current time slice that we are during the execution of the
       //algorithm, which simulates a working day
       int currentTimeSlice = 1;
       boolean threadStopped = false, isNewNodesAvailable = false, isNewNodesCommitted = false;
       //keeps the last index/position in the array of dynamic requests sorted ascending by available time
       //of the recent request which became available in the last time slice  
       int countApriori, lastPos;
       ArrayList<Integer> newAvailableIdNodes = new ArrayList<Integer>();
       ArrayList<Integer> idKnownRequests = new ArrayList<Integer>();
       ArrayList<Integer> lastCommitedIndexes;
       double sum;
       
       for (int trial = 0; trial < 30; trial++) {
    	 //reads benchmark data; read the data from the input file
    	 String dvrptwInstance = vrpInstance + "-" + dynamicLevel;  
    	 String fileName = dvrptwInstance + ".txt";
		 DataReader reader = new DataReader(fileName);
         //read the data from the file
         VRPTW vrpInstance = reader.read();
	        
         System.out.println("DVRPTW_ACS MinSum >> Solving dynamic VRPTW instance: " + dvrptwInstance);
         //include in the counting also the depot, which is assumed to be apriori known
         countApriori = vrpInstance.getIdAvailableRequests().size();
         System.out.println("No. of customers' requests (except the depot): " +  VRPTW.n + ", among which " + countApriori + " are apriori known (available nodes excluding the depot) and " + vrpInstance.getDynamicRequests().size() + " are dynamic requests");  
		 
		 //compute the scaling value with which we can scale all time-related values
		 Request depotReq = vrpInstance.getRequests().get(0);
		 scalingValue = (double)workingDay / (double)(depotReq.getEndWindow() - depotReq.getStartWindow());
		 Controller.setScalingValue(scalingValue);
		 
		 //adjust distances between nodes (cities) according to this scale value
		 InOut.init_program(args, trial, vrpInstance, scalingValue);
		 
		 //adjust for each request, all the time related values according to the length of the working day we are simulating
		 if (scalingValue != 0) {
			System.out.println("Scalling value = " + scalingValue); 
			for (Request req : vrpInstance.getRequests()) {
				 newStartWindow = req.getStartWindow() * scalingValue;
				 req.setStartWindow(newStartWindow);
				 newEndWindow = req.getEndWindow() * scalingValue;
				 req.setEndWindow(newEndWindow);
				 newServiceTime = req.getServiceTime() * scalingValue;
				 req.setServiceTime(newServiceTime);
				 newAvailableTime = req.getAvailableTime() * scalingValue;
				 req.setAvailableTime(newAvailableTime);
			 } 
		 }
		 
		 //sorting dynamic requests in ascending order by their available time
         //System.out.println("Sorted list of dynamic requests..");
		 ArrayList<Request> dynamicRequests = vrpInstance.getDynamicRequests();
		 Collections.sort(dynamicRequests);
		 vrpInstance.setDynamicRequests(dynamicRequests);
		 /*for (Request req: dynamicRequests) {
			System.out.println("ID: " + req.getId() + " XCoord: " + req.getxCoord() + " YCoord: " + req.getyCoord() + 
					 " demand: " + req.getDemand() + " startWindow: " + req.getStartWindow() + " endWindow: " + req.getEndWindow() +
					 " service time: " + req.getServiceTime() + " available time: " + req.getAvailableTime());
		 }*/
		 
		 int[][][] result = new int[2][][];
		 result = VRPTW.compute_nn_lists(vrpInstance);
		 VRPTW.instance.nn_list = result[0];
		 VRPTW.instance.nn_list_all = result[1];
		
		 Ants.pheromone = new double[VRPTW.n + 1][VRPTW.n + 1];
		 //Ants.total = new double[MTsp.n + 1][MTsp.n + 1];
		 
		 //VRPTW_ACS.generateInitialWeights();
		 VRPTW_ACS.init_try(vrpInstance); 
   
		 currentTimeSlice = 1;
		 idLastAvailableNode = 0;
		 addedNodes = 0;
		 InOut.noEvaluations = 0;
		 InOut.noSolutions = 0;
         double lengthTimeSlice = (double)workingDay / (double)noTimeSlices;
         startTime = System.currentTimeMillis();
       
	     //start the ant colony
	     VRPTW_ACS worker = new VRPTW_ACS(threadStopped, vrpInstance);
		 Thread t = new Thread(worker);
		 t.start();
    	   
		 //check periodically if the problem has changed and new nodes (customer requests) became available
		 //or there are nodes from the best so far solution that must be marked as committed
    	 do {
    	   //compute current time up to this point
    	   endTime = System.currentTimeMillis();
    	   currentTime = (endTime - startTime)/1000.0;
    	   /*if (currentTimeSlice >= 30) {
    		   System.out.println("Trial " + (trial + 1) + " Before if: computed current time=" + currentTime + " currentTimeSlice=" + currentTimeSlice + " lengthTimeSlice=" + lengthTimeSlice);	   
    	   }*/
    	   //did a new time slice started?
    	   if (currentTime > currentTimeSlice * lengthTimeSlice) {
			   //advance to next time slice
    		   System.out.println("Trial " + (trial + 1) + "; Current time (seconds): " + currentTime + "; new time slice started at " + currentTimeSlice * lengthTimeSlice);   
			   //check if there are new nodes that became available in the last time slice
			   newAvailableIdNodes = countNoAvailableNodes(dynamicRequests, currentTime);
			   //mark the fact that new nodes (from the list of dynamic customer requests) are available 
			   int countNodes = newAvailableIdNodes.size();
			   if (countNodes > 0) {
				   isNewNodesAvailable = true;
			   }
			   else {
				   isNewNodesAvailable = false;
			   }
			   //check if there are nodes that must be marked as committed in the tours of the best so far solution
			   /*if (currentTimeSlice >= 30) {
				  System.out.println("Before checking for new nodes to be committed..; isNewNodesAvailable="  + isNewNodesAvailable); 
			   }*/
			   isNewNodesCommitted = checkNewCommittedNodes(Ants.best_so_far_ant, vrpInstance, currentTimeSlice, lengthTimeSlice);
			   /*if (currentTimeSlice >= 30) {
				  System.out.println("After checking for nodes to be committed..isNewNodesAvailable="  + isNewNodesAvailable + " isNewNodesCommitted=" + isNewNodesCommitted); 
			   }*/
			   //check if new nodes are available (known) or there are parts (nodes) that must be committed from the tours of the best so far solution
			   if (isNewNodesAvailable || isNewNodesCommitted) {
				   //System.out.println("Need to stop ant colony..isNewNodesAvailable=" + isNewNodesAvailable + " isNewNodesCommitted=" + isNewNodesCommitted);
				   //stop the execution of the ant colony thread
				   if (t != null) {
	    			   worker.terminate();
		    		   //wait for the thread to stop
		    		   try {
		    			   t.join();
					   } catch (InterruptedException e) {
						   e.printStackTrace();
					   }
	    		   }      			
	    		   //System.out.println("CurrentTimeSlice = " + currentTimeSlice + ": Stopping the worker thread");
	    		   threadStopped = true;
	    		   //System.out.println("After stopping ant colony..isNewNodesAvailable=" + isNewNodesAvailable + " isNewNodesCommitted=" + isNewNodesCommitted);
	    		   //if there are nodes to be committed
	    		   if (isNewNodesCommitted) {
	    			  /* if (currentTimeSlice >= 30) {
	    				   System.out.println("Before nodes were committed.." + "After stopping ant colony..isNewNodesAvailable=" + isNewNodesAvailable + " isNewNodesCommitted=" + isNewNodesCommitted);
	    			   }*/
	    			   //commit necessary nodes after the ant colony execution is stopped	    			  
	    			   commitNodes(Ants.best_so_far_ant, vrpInstance, currentTimeSlice, lengthTimeSlice);
	    			  /* if (currentTime >= 20) {
	    				   System.out.println("After nodes were committed.." + "After stopping ant colony..isNewNodesAvailable=" + isNewNodesAvailable + " isNewNodesCommitted=" + isNewNodesCommitted);
	    			   }*/
	    		   }
				   
			   }
			   
			   //if there are new available nodes, update the list of available/known nodes (customer requests)
			   if (isNewNodesAvailable) {
				   System.out.print(countNodes + " new nodes became available (known): ");
				   idKnownRequests = vrpInstance.getIdAvailableRequests();
				   for (int id : newAvailableIdNodes) {
					   idKnownRequests.add(id);
					   System.out.print((id + 1) + " ");
				   }
				   vrpInstance.setIdAvailableRequests(idKnownRequests);
				   System.out.println();
				   System.out.println("Number of total available (known) nodes (excluding the depot): " + idKnownRequests.size());
				   
				   //insert new available nodes in the best so far solution
				   Ants.best_so_far_ant.toVisit = countNodes;
				   //determine nodes that are not visited yet in the current ant's solution
				   ArrayList<Integer> unroutedList = Ants.unroutedCustomers(Ants.best_so_far_ant, vrpInstance);
				   //skip over committed (defined) nodes when performing insertion heuristic
				   lastCommitedIndexes = new  ArrayList<Integer>();
				   for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
					   lastPos = getLastCommitedPos(index);
					   lastCommitedIndexes.add(lastPos);
				   }
				   InsertionHeuristic.insertUnroutedCustomers(Ants.best_so_far_ant, vrpInstance, unroutedList, 0, lastCommitedIndexes);
				   //System.out.println("After first applying insertion heuristic: Cities to be visited in the best so far solution: " + Ants.best_so_far_ant.toVisit);
				   //if there are still remaining unvisited cities from the ones that are available
				   //insert an empty tour and add cities in it following nearest-neighbour heuristic
				   int indexTour;
				   while (Ants.best_so_far_ant.toVisit > 0) {
					    Ants.best_so_far_ant.usedVehicles++;
						indexTour = Ants.best_so_far_ant.usedVehicles - 1;
						Ants.best_so_far_ant.tours.add(indexTour, new ArrayList<Integer>());
						Ants.best_so_far_ant.tours.get(indexTour).add(-1); 
						Ants.best_so_far_ant.tour_lengths.add(indexTour, 0.0);
						Ants.best_so_far_ant.currentQuantity.add(indexTour, 0.0);
						Ants.best_so_far_ant.currentTime.add(indexTour, 0.0);
						//Ants.lastCommitted.add(indexTour, 0);
						
						//try to add as many unvisited cities/nodes as possible in this newly created tour
						//following the nearest neighbour heuristic
						Ants.choose_closest_nn(Ants.best_so_far_ant, indexTour, vrpInstance);
						//System.out.println("After adding new tour & NN tour construction: Cities to be visited in the best so far solution: " + Ants.best_so_far_ant.toVisit);
						
						//try to insert remaining cities using insertion heuristic
						if (Ants.best_so_far_ant.toVisit > 0) {
							 //determine nodes that are not visited yet in the current ant's solution
							 unroutedList = Ants.unroutedCustomers(Ants.best_so_far_ant, vrpInstance);
							 //skip over committed (defined) nodes when performing insertion heuristic
							 lastCommitedIndexes = new  ArrayList<Integer>();
							 for (int index = 0; index < Ants.best_so_far_ant.usedVehicles; index++) {
								 lastPos = getLastCommitedPos(index);
								 lastCommitedIndexes.add(lastPos);
							 }
							 InsertionHeuristic.insertUnroutedCustomers(Ants.best_so_far_ant, vrpInstance, unroutedList, indexTour, lastCommitedIndexes);
							 //System.out.println("After applying insertion heuristic to the NN tour: Cities to be visited in the best so far solution: " + Ants.best_so_far_ant.toVisit);
						}	
						//add the depot again to end this tour
						Ants.best_so_far_ant.tours.get(indexTour).add(-1); 
				   }
				   /*if (VRPTW_ACS.ls_flag) {
					   Ants.best_so_far_ant = VRPTW_ACS.local_search(Ants.best_so_far_ant, vrpInstance);
				   }*/
				   
				   sum = 0.0;
				   for (int i = 0; i < Ants.best_so_far_ant.usedVehicles; i++) {
					   Ants.best_so_far_ant.tour_lengths.set(i, VRPTW.compute_tour_length_(Ants.best_so_far_ant.tours.get(i)));
					   sum += Ants.best_so_far_ant.tour_lengths.get(i);	
				   }
				   Ants.best_so_far_ant.total_tour_length = sum;
				   
				   scalingValue = Controller.getScalingValue();
			       double scalledValue = 0.0;
				   if (scalingValue != 0) {
					   scalledValue = Ants.best_so_far_ant.total_tour_length / scalingValue;
				   }
				   System.out.println("Best ant after inserting the new available nodes>> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length + " (scalled value = " + scalledValue + ")");
				   /*for (int i = 0; i < Ants.best_so_far_ant.usedVehicles; i++) {
						int tourLength = Ants.best_so_far_ant.tours.get(i).size();
						for (int j = 0; j < tourLength; j++) {
							int city = Ants.best_so_far_ant.tours.get(i).get(j);
							city = city + 1;  //so as to correspond to the city indexes from the VRPTW input file
							System.out.print(city + " ");		
						}
						System.out.println();
				  }*/
			   }
		        
        	   currentTimeSlice++;
           }
        	   
    	   //restart the colony thread
   		   if (threadStopped) {
   			  //restart the ant colony thread
   			  worker = new VRPTW_ACS(threadStopped, vrpInstance);
   		      t = new Thread(worker);
   		      t.start(); 
   		      threadStopped = false;
       	   }
       		   
   		   // sleep for a period
   	       // 5 times within each time step of the working day we are simulating
		   /*try {
			   Thread.sleep(time);
		   } catch (InterruptedException e) {
			   e.printStackTrace();
		   }*/
   		  /* if (currentTimeSlice >= 30) {	  
   			   System.out.println("Checking end of loop: current time=" + currentTime + " currentTimeSlice=" + currentTimeSlice + " lengthTimeSlice=" + lengthTimeSlice);
   		   }*/
   		   if (currentTime >= workingDay) {
	    	  break;
	       }
    	       
         } while(true);
           
         //working day is over
    	//System.out.println("End of working day.." + currentTime);
        //stop the worker thread
        if (t != null) {
    	  worker.terminate();
		  //wait for the thread to stop
		  try {
			  t.join();
		  } catch (InterruptedException e) {
			  e.printStackTrace();
		  } 
        }
        
        //end of the working day; try final improvements of the best so far solution
        //by applying iterated relocate multiple route and exchange multiple route local search operators 
       /* if (VRPTW_ACS.ls_flag) {
        	Ants.best_so_far_ant = VRPTW_ACS.relocateMultipleRouteIterated(Ants.best_so_far_ant, vrpInstance);
        	Ants.best_so_far_ant = VRPTW_ACS.exchangeMultipleRouteIterated(Ants.best_so_far_ant, vrpInstance);
		}*/
        
        scalingValue = Controller.getScalingValue();
        double scalledValue = 0.0;
		if (scalingValue != 0) {
			scalledValue = Ants.best_so_far_ant.total_tour_length / scalingValue;
		}
	    System.out.println("Final best solution >> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length + " (scalled value = " + scalledValue + ")");
	    for (int i = 0; i < Ants.best_so_far_ant.usedVehicles; i++) {
			int tourLength = Ants.best_so_far_ant.tours.get(i).size();
			for (int j = 0; j < tourLength; j++) {
				int city = Ants.best_so_far_ant.tours.get(i).get(j);
				city = city + 1;  //so as to correspond to the city indexes from the VRPTW input file
				System.out.print(city + " ");		
			}
			System.out.println();
		}   
	    System.out.println("Total number of evaluations: " + InOut.noEvaluations);
	    System.out.println("Total number of feasible solutions: " + InOut.noSolutions);
        //System.out.println("Working day is over..");
	    boolean isValid = Utilities.checkFeasibility(Ants.best_so_far_ant, vrpInstance, true);
	    if (isValid) {
	    	System.out.println("The final solution is valid (feasible)..");
	    }
	    else {
	    	System.out.println("The final solution is not valid (feasible)..");
	    }
	    
	    //save final solution in a .txt file on the disk
	    Utilities.writeFinalSolution(trial, fileName, scalledValue, isValid);
	    
	    //save number of used vehicles and total traveled distance in Excel file
	    Utilities.writeExcelFinalSolution(trial, scalledValue);
	    
       } 

	}

}
