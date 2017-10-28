package autopilot;

import p_en_o_cw_2017.*;
import javax.swing.*;
import java.io.*;

<<<<<<< HEAD
    public void fillStreamWithOutput(java.io.DataInputStream inputStream, java.io.DataOutputStream outputStream) {
        AutopilotInputs input;
        AutopilotOutputs output;
        try {
            input = AutopilotInputsReader.read(inputStream);
            if (this.previousInput==null) {
                output = new AutopilotOutputs();
            }
            else {
                output = InputToOutput.calculate(input,ImageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns(), this);

            }
            try {
                AutopilotOutputsWriter.write(outputStream, output);
                this.userInterface.updateData(output);
            } catch(IOException e) {
                e.printStackTrace();
            }
            this.previousInput = input;
            this.previousOutput = output;
        } catch (IOException e) {
            e.printStackTrace();
        }
=======
public class Autopilot implements p_en_o_cw_2017.Autopilot {
>>>>>>> 00d14e58b126e5ff363ca8c17461c20b667e3160

    private boolean isSimulating = false; // TODO: never used for the moment
    private AutopilotConfig config;
    private AutopilotInputs previousInput;
    private AutopilotOutputs previousOutput;
    UI userInterface = new UI();

    AutopilotInputs getPreviousInput() {
        return this.previousInput;
    }

    AutopilotOutputs getPreviousOutput() {
        return this.previousOutput;
    }

    AutopilotConfig getConfig() {
        return this.config;
    }

    private void setConfig(AutopilotConfig config) {
        this.config = config;
    }

    @Override
    public p_en_o_cw_2017.AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
        this.setConfig(config);
        this.isSimulating = true;
        // TODO: Is this the correct way to handle the incoming inputs?
        return timePassed(inputs);
    }

    @Override
    public p_en_o_cw_2017.AutopilotOutputs timePassed(AutopilotInputs inputs) {
        AutopilotOutputs output;

        if (this.previousInput == null) {
            output = new AutopilotOutputs();
        } else {
            output = InputToOutput.calculate(inputs, ImageRecognition.FindTarget(inputs.getImage(), config.getNbColumns(), config.getNbRows()), config.getNbRows(), config.getNbColumns(), this);
        }
        this.previousInput = inputs;
        this.previousOutput = output;

        return output;
    }

    @Override
    public void simulationEnded() {
        this.isSimulating = false;
    }
}
