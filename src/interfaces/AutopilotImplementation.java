package interfaces;

public class AutopilotImplementation implements Autopilot {

    private boolean isSimulating = false; // TODO: never used for the moment
    private AutopilotConfig config;
    private PreviousInputs previousInput;
    private AutopilotOutputsImplementation previousOutput;
    private Drone drone;
    UI userInterface = new UI();
    private AutopilotOutputs move;
    private Job job;

    public AutopilotImplementation(Airport airport, int gate, int pointingToRunway, AutopilotConfig config) {
    	this.drone = new Drone(airport, gate, pointingToRunway);
    	this.config = config;
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
    
    public Job getJob() {
    	return this.job;
    }
    
    public void setJob(Job job) {
    	if (! this.hasJob()) {
    		this.job = job;
    	}
    }
    
    public boolean hasJob() {
    	return (! (this.getJob() == null));
    }
}
