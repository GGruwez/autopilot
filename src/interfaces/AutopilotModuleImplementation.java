package interfaces;

import java.util.ArrayList;
import java.util.Dictionary;

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

    	AutopilotImplementation closestDrone = null;
		float closestDistance = Float.MAX_VALUE;
		AutopilotImplementation idleDrone = null;
		float minJobs = MAX_NB_JOBS;
    	
    	for (AutopilotImplementation drone : this.getDrones()) {
    		
    		// Vrije drone op vertrekplaats
    		if (drone.getDrone().getAirport() == job.getAirportFrom() &&
    				drone.getDrone().getGate() == job.getGateFrom() &&
    				(! drone.hasJob())) {
    			drone.addJob(job);
    			job.setDrone(drone);
    		}
    		
    		else {
    			// Dichtstbijzijnde drone zoeken --> vind je sowieso
    			if (drone.getDistanceToAirport(job.getAirportFrom()) < closestDistance) {
	    			closestDrone = drone;
	    			closestDistance = drone.getFinalAirport().getDistanceToAirport(job.getAirportFrom());
	    		}
    		
    			// Drone met minste jobs onder max aantal --> vind je niet sowieso als alle drones vol zijn
	    		else if (drone.getJobs().size() < minJobs) {
	    			idleDrone = drone;
	    			minJobs = drone.getJobs().size();
	    		}
    		}
    	}
		
    	if (! job.hasDrone()) {
			if (idleDrone != null) {
				idleDrone.addJob(job);
				job.setDrone(idleDrone);
			}
			
			else {
				if (closestDrone.getJobs().size() < MAX_NB_JOBS) {
					closestDrone.addJob(job);
					job.setDrone(closestDrone);
				}
				else {
					MAX_NB_JOBS += 1;
					assignJob(job);
				}
			}
    	}
    }
    
    public float airportLength = 1000;
    public float airportWidth = 50;
    public ArrayList<Airport> airports = new ArrayList<Airport>();
    public ArrayList<AutopilotImplementation> drones = new ArrayList<AutopilotImplementation>();
    public ArrayList<Job> jobs = new ArrayList<Job>();
    public int MAX_NB_JOBS = 3;
}