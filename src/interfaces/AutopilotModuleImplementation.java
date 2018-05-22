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
    	drone.setModule(this);
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
    	
    	// Collision detection
    	for (AutopilotImplementation drone: this.getDrones()) {
    		boolean bool = false;
    		if (drone.hasJob() && job.getDrone() != drone) {
	    		for (Vector cube1: drone.getCurrentPath().getArrayList()) {
	    			for (Vector cube2: job.getDrone().getCurrentPath().getArrayList()) {
	    				if (Math.sqrt(Math.pow(cube1.getX() - cube2.getX(),2) + Math.pow(cube1.getZ() - cube2.getZ(),2)) < 100
								&& Math.abs(cube1.getY()-cube2.getY()) < 10) {
	    					bool = true;
	    				}
	    			}
	    		}
	    		if (bool) {
	    			job.getDrone().getCurrentPath().collisionUpdate();
	    		}
    		}
    	}
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
		AutopilotImplementation assignedDrone = null;
    	
    	for (AutopilotImplementation drone : this.getDrones()) {
    		
    		// Vrije drone op vertrekplaats
    		if (drone.getDrone().getAirport() == job.getAirportFrom() &&
    				drone.getDrone().getGate() == job.getGateFrom() &&
    				(! drone.hasJob())) {
    			assignedDrone = drone;
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
		
    	if (assignedDrone == null) {
			if (idleDrone != null) {
				assignedDrone = idleDrone;
			}
			
			else {
				if (closestDrone.getJobs().size() < MAX_NB_JOBS) {
					assignedDrone = closestDrone;
				}
				else {
					MAX_NB_JOBS += 1;
					assignJob(job);
					return;
				}
			}
    	}
    	
    	if (!assignedDrone.hasJob() && ((assignedDrone.getDrone().getAirport() != job.getAirportFrom()) ||
    			(assignedDrone.getDrone().getGate() != job.getGateFrom()))){
    		Log.println("tussenstop");
    		Job tussenstop = new Job(assignedDrone.getDrone().getAirport(),assignedDrone.getDrone().getGate(),
    				job.getAirportFrom(),job.getGateFrom());
    		tussenstop.setDrone(assignedDrone);
    		assignedDrone.addJob(tussenstop);
    	}
    	else if (assignedDrone.hasJob() &&(assignedDrone.getJobs().get(assignedDrone.getJobs().size()-1).getAirportTo() != job.getAirportFrom() ||
				assignedDrone.getJobs().get(assignedDrone.getJobs().size()-1).getGateTo() != job.getGateFrom())) {
			Log.println("tussenstop");
			Job tussenstop = new Job(assignedDrone.getDrone().getAirport(),assignedDrone.getDrone().getGate(),
					job.getAirportFrom(),job.getGateFrom());
			tussenstop.setDrone(assignedDrone);
			assignedDrone.addJob(tussenstop);
		}
		job.setDrone(assignedDrone);
		assignedDrone.addJob(job);
    }
    
    public float airportLength = 650;
    public float airportWidth = 60;
    public ArrayList<Airport> airports = new ArrayList<Airport>();
    public ArrayList<AutopilotImplementation> drones = new ArrayList<AutopilotImplementation>();
    public ArrayList<Job> jobs = new ArrayList<Job>();
    public int MAX_NB_JOBS = 3;
}
