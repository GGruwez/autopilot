package autopilot;

import p_en_o_cw_2017.*;
import javax.swing.*;
import java.io.*;
import src.autopilot.*;

/**
 * Created by ndbae06 on 16/10/2017.
 */
public class Autopilot {
	
    AutopilotConfigReader configReader;
    AutopilotConfig config;
    AutopilotInputs previousInput;
    AutopilotOutputs previousOutput;
    UI userInterface = new UI();

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
            } catch(IOException e) {
                e.printStackTrace();
            }
            this.previousInput = input;
            this.previousOutput = output;
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public AutopilotInputs getPreviousInput() {
        return this.previousInput;
    }

    public AutopilotOutputs getPreviousOutput() {
        return this.previousOutput;
    }
    
    public AutopilotConfig getConfig() {
    	return this.config;
    }
    
    public void setConfig(AutopilotConfig config) {
    	this.config = config;
    }

}
