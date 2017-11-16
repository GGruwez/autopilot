package interfaces;

public class AutopilotImplementation implements Autopilot {

    private boolean isSimulating = false; // TODO: never used for the moment
    private AutopilotConfig config;
    private PreviousInputs previousInput;
    private AutopilotOutputsImplementation previousOutput;
    UI userInterface = new UI();

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
        AutopilotOutputsImplementation output;

        if (!isSimulating) {
            output = new AutopilotOutputsImplementation();
            this.isSimulating = true;
        } else {
            output = InputToOutput.calculate(inputs, ImageRecognition.FindTarget(inputs.getImage(), config.getNbColumns(), config.getNbRows()), config.getNbRows(), config.getNbColumns(), this);
            this.userInterface.updateData(output);
        }
        this.previousInput = new PreviousInputs(inputs);
        this.previousOutput = output;

        return output;
    }

    @Override
    public void simulationEnded() {
        this.isSimulating = false;
    }
}
