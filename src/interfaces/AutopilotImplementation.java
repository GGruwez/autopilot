package interfaces;

import java.util.ArrayList;

public class AutopilotImplementation implements Autopilot {
	
	//TODO: een drone hoort altijd bij de airport waar hij naartoe aan het vliegen is, 
	//		of waar hij op stilstaat.
	//TODO: variabele 'landed' zetten als hij landt, niet zetten als hij opstijgt. 

    private boolean isSimulating = false; // TODO: never used for the moment
    private AutopilotConfig config;
    private PreviousInputs previousInput;
    private AutopilotOutputsImplementation previousOutput;
    private Drone drone;
    UI userInterface = new UI();
    private AutopilotOutputs move;

    //testing
	Airport airport1 = new Airport(0,0,0,-1);
	Airport airport2 = new Airport(4000,0,0,-1);
	Job job = new Job(airport1,0,airport2,0);
	//testing
	private ArrayList<Job> jobs = new ArrayList<Job>();
    private boolean landed = true;
    private AutopilotModuleImplementation module;

    public AutopilotImplementation(Airport airport, int gate, int pointingToRunway, AutopilotConfig config) {
    	this.drone = new Drone(airport, gate, pointingToRunway);
    	this.config = config;
    	airport.setDroneAt(gate, this);
    	//todo remove this
		 jobs.add(job);

    }
    
  	public boolean isLanded() {
  		return this.landed;
  	}
  	
  	public AutopilotModuleImplementation getModule() {
  		return this.module;
  	}
  	
  	public void setModule(AutopilotModuleImplementation module) {
  		this.module = module;
  	}
  	
  	public void setLanded(boolean bool) {
  		this.landed = bool;
  	}
    
    PreviousInputs getPreviousInput() {
        return this.previousInput;
    }

    AutopilotOutputsImplementation getPreviousOutput() {
        return this.previousOutput;
    }

    AutopilotConfig getConfig() {
        return this.config;
    }

    private void setConfig(AutopilotConfig config) {
        this.config = config;
    }

    @Override
    public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
        this.setConfig(config);
        AutopilotOutputs output = new AutopilotOutputsImplementation();//timePassed(inputs);
        
        // TODO: Is this the correct way to handle the incoming inputs?
        return output;
    }

    @Override
    public AutopilotOutputs timePassed(AutopilotInputs inputs) {
        AutopilotOutputsImplementation output = new AutopilotOutputsImplementation(0,0,0,0,0,0,0,0);
        System.out.println("no job");
        if (this.hasJob()) {
	        if (!isSimulating) {
	            output = new AutopilotOutputsImplementation();
	            this.isSimulating = true;
	        } else {
	            output = drone.calculate(inputs,null, config.getNbRows(), config.getNbColumns(), this);
	            System.out.println("running");
	            this.userInterface.updateData(output);
	        }
	        this.previousInput = new PreviousInputs(inputs);
	        this.previousOutput = output;
        }

        return output;
    }

    @Override
    public void simulationEnded() {
        this.isSimulating = false;
    }
    
    public void setMove(AutopilotInputs inputs) {
    	this.move = this.timePassed(inputs);
    }
    
    public AutopilotOutputs getMove() {
    	return this.move;
    }
    
    public Job getCurrentJob() {
    	if (! this.hasJob()) {
    		return null;
    	}
    	Airport idleAirport = null;
		int idleGate = 0;
    	if (this.getJobs().get(0).getAirportTo().hasDroneAt(this.getJobs().get(0).getGateTo())) {
    		AutopilotImplementation drone = this.getJobs().get(0).getAirportTo().getDroneAt(this.getJobs().get(0).getGateTo());
    		if (! drone.hasJob()) {
    			for (Airport airport: getModule().getAirports()) {
    				if (! airport.hasDroneAt(0)) {
    					idleAirport = airport;
    					idleGate = 0;
    				}
    				else if (! airport.hasDroneAt(1)) {
    					idleAirport = airport;
    					idleGate = 1;
    				}
    			}
	    		Job newJob = new Job(drone.getDrone().getAirport(),drone.getDrone().getGate(),idleAirport, idleGate);
	        	System.out.println("idleairport null?" + idleAirport == null);
	        	drone.addJob(newJob);
	        	newJob.setDrone(drone);
    		}
    	}
    	return this.getJobs().get(0);
    }
    
    public ArrayList<Job> getJobs() {
    	return this.jobs;
    }
    
    public void addJob(Job job) {
    	this.jobs.add(job);
    }
    
    public Drone getDrone() {
    	return this.drone;
    }
    
    public boolean hasJob() {
    	return (! (this.getJobs().size() == 0));
    }
    
    public float getDistanceToAirport(Airport airport) {
    	return (float) Math.sqrt(
    			Math.pow((getDrone().getAirport()).getCenterX()-airport.getCenterX(),2) +
    			Math.pow((getDrone().getAirport()).getCenterZ()-airport.getCenterZ(),2)
    			);
    }
    
    public Airport getFinalAirport() {
    	if (! hasJob()) {
    		return this.getDrone().getAirport();
    	}
    	return this.getJobs().get(this.getJobs().size()-1).getAirportTo();
    }

    public PathImplementation getCurrentPath() {
    	if (! hasJob()) {
    		return null;
    	}
    	return this.getCurrentJob().getPath();
    }
    
    public void finishCurrentJob() {
    	getCurrentJob().setDrone(null);
    	this.getJobs().remove(getCurrentJob());
    }
}
