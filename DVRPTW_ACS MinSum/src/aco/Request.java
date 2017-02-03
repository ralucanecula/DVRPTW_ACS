package aco;

//models a request from a client that contains certain information
//it implements Comparable interface so that we can sort an arraylist of requests according to the natural order of objects
public class Request implements Comparable<Request> {
    /** id of the client request */
    private int id;

	/** x coordinate of the request */
    private int xCoord;
    
    /** y coordinate of the request */
    private int yCoord;
	
    /** demand (delivery quantity) of the request */
    private int demand;
    
    /** ready time of the request; earliest arrival time/lower limit/beginning of the time window */
    private double startWindow;
    
    /** due date of the request; latest arrival time/upper bound/end of the time window */
    private double endWindow;
    
    /** service time/duration (necessary for unloading/loading of the goods) corresponding to the client request
     * service time involves pickup and/or delivery of goods or services for s_i units of time
     A service time s_i is associated with each customer. It represents the time required to service him/her or the
     time spend by the vehicle at the customer once it arrived at its location before moving/traveling to the next customer
     */
    private double serviceTime;
    
    /** it represents the moment in time when the customer request is known to the system and becomes
     *  available as node in the optimization task of ACS as part of the DVRPTW instance being solved */
    private double availableTime;
    
    
    public Request() {}
    
    public Request (int id_, int xCoord_, int yCoord_, int demand_, int startWindow_, int endWindow_, int serviceTime_, int availableTime_) {
    	this.id = id_;
    	this.xCoord = xCoord_;
    	this.yCoord = yCoord_;
    	this.demand = demand_;
    	this.startWindow = startWindow_;
    	this.endWindow = endWindow_;
    	this.serviceTime = serviceTime_;
    	this.availableTime = availableTime_;
    }
    
    public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public int getxCoord() {
		return xCoord;
	}

	public void setxCoord(int xCoord) {
		this.xCoord = xCoord;
	}

	public int getyCoord() {
		return yCoord;
	}

	public void setyCoord(int yCoord) {
		this.yCoord = yCoord;
	}

	public int getDemand() {
		return demand;
	}

	public void setDemand(int demand) {
		this.demand = demand;
	}

	public double getStartWindow() {
		return startWindow;
	}

	public void setStartWindow(double startWindow) {
		this.startWindow = startWindow;
	}

	public double getEndWindow() {
		return endWindow;
	}

	public void setEndWindow(double endWindow) {
		this.endWindow = endWindow;
	}

	public double getServiceTime() {
		return serviceTime;
	}

	public void setServiceTime(double serviceTime) {
		this.serviceTime = serviceTime;
	}

	public double getAvailableTime() {
		return availableTime;
	}

	public void setAvailableTime(double availableTime) {
		this.availableTime = availableTime;
	}

	@Override
	public int compareTo(Request req) {
		int result;
			
		Double time = (Double)this.getAvailableTime();
		Double otherTime = (Double)req.getAvailableTime();
		
		result = time.compareTo(otherTime);
		
	    return result;
		
	}
    
}
