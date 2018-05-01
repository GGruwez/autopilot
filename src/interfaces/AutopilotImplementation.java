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
    private ArrayList<Job> jobs;
    private Airport[] currentPath;
    private int[] currentPathGates;
    private boolean landed = false;

    public AutopilotImplementation(Airport airport, int gate, int pointingToRunway, AutopilotConfig config) {
    	this.drone = new Drone(airport, gate, pointingToRunway);
    	this.config = config;
    }
    
  	public boolean isLanded() {
  		return this.landed;
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
        
        if (this.hasJob()) {
	        if (!isSimulating) {
	            output = new AutopilotOutputsImplementation();
	            this.isSimulating = true;
	        } else {
	            output = drone.calculate(inputs,null, config.getNbRows(), config.getNbColumns(), this);
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

    public Airport[] getCurrentPath() {
    	if (! hasJob()) {
    		this.currentPathGates = null;
    		return null;
    	}
    	if (! isLanded()) {
    		return this.currentPath;
    	}
    	Job currentJob = this.getCurrentJob();
    	Airport from = this.getDrone().getAirport();
    	int fromGate = this.getDrone().getGate();
    	Airport to = currentJob.getAirportFrom();
    	int toGate = currentJob.getGateFrom();
    	if (currentJob.getAirportFrom() == from) {
    		to = currentJob.getAirportTo();
    		toGate = currentJob.getGateTo();
    	}
    	this.currentPathGates = new int[]{fromGate,toGate};
    	this.currentPath = new Airport[]{from,to};
    	return this.currentPath;
    }
    
    public int[] getCurrentPathGates() {
    	if (! hasJob()) {
    		this.currentPathGates = null;
    		return null;
    	}
    	if (! isLanded()) {
    		return this.currentPathGates;
    	}
    	Job currentJob = this.getCurrentJob();
    	Airport from = this.getDrone().getAirport();
    	int fromGate = this.getDrone().getGate();
    	Airport to = currentJob.getAirportFrom();
    	int toGate = currentJob.getGateFrom();
    	if (currentJob.getAirportFrom() == from) {
    		to = currentJob.getAirportTo();
    		toGate = currentJob.getGateTo();
    	}
    	this.currentPathGates = new int[]{fromGate,toGate};
    	this.currentPath = new Airport[]{from,to};
    	return this.currentPathGates;
    }
}
