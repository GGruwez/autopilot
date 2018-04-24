package interfaces;

import java.util.ArrayList;

public class AutopilotModuleImplementation implements AutopilotModule {
	public void defineAirportParams(float length, float width) {
		this.airportLength = length;
		this.airportWidth = width;
	}
	
    public void defineAirport(float centerX, float centerZ, float centerToRunway0X, float centerToRunway0Z) {
    	Airport airport = new Airport(centerX, centerZ, centerToRunway0X, centerToRunway0Z);
    	this.airports.add(airport);
    }
    
    public void defineDrone(int airport, int gate, int pointingToRunway, AutopilotConfig config) {
    	AutopilotImplementation drone = new AutopilotImplementation(getAirport(airport), gate, pointingToRunway, config);
    	this.drones.add(drone);
    }
    	
    public void startTimeHasPassed(int drone, AutopilotInputs inputs){
    	this.getDrone(drone).setMove(inputs);
    }
    
    public AutopilotOutputs completeTimeHasPassed(int drone){
    	return this.getDrone(drone).getMove();
    }
    
    public void deliverPackage(int fromAirport, int fromGate, int toAirport, int toGate) {
    	Job job = new Job(this.getAirport(fromAirport), fromGate, this.getAirport(toAirport), toGate);
    	this.jobs.add(job);
    	this.assignJob(job);
    }
    
    public void simulationEnded() {
    	for (int i = 0; i < this.getDrones().size(); i++) {
    		this.getDrone(i).simulationEnded();
    	}
    }
    
    public float getAirportLength() {
    	return this.airportLength;
    }
    
    public float getAirportWidth() {
    	return this.airportWidth;
    }
    
    public Airport getAirport(int airport) {
    	return this.airports.get(airport);
    }
    
    public ArrayList<Airport> getAirports() {
    	return this.airports;
    }
    
    public AutopilotImplementation getDrone(int drone) {
    	return this.drones.get(drone);
    }
    
    public ArrayList<AutopilotImplementation> getDrones() {
    	return this.drones;
    }
    
    public ArrayList<Job> getJobs() {
    	return this.jobs;
    }
    
    public void assignJob(Job job) {
    	//TODO: job toewijzen aan drone
    }
    
    public float airportLength;
    public float airportWidth;
    public ArrayList<Airport> airports;
    public ArrayList<AutopilotImplementation> drones;
    public ArrayList<Job> jobs;
}
