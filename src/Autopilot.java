

import p_en_o_cw_2017.*;

import java.io.*;

/**
 * Created by ndbae06 on 16/10/2017.
 */
public class Autopilot {
    ImageRecognition imageRecognition;
    InputToOutput inputToOutput;
    AutopilotConfigReader configReader;
    AutopilotInputsReader reader;
    AutopilotOutputsWriter writer;
    AutopilotConfig config;
    AutopilotInputs previousInput;
    AutopilotOutputs previousOutput;

    public Autopilot(DataInputStream configstream) {
        this.imageRecognition = new ImageRecognition();
        this.inputToOutput = new InputToOutput();
        this.reader = new AutopilotInputsReader();
        this.writer = new AutopilotOutputsWriter();
        this.configReader = new AutopilotConfigReader();
        try { this.config = this.configReader.read(configstream);}
        catch (IOException e) {e.printStackTrace();}
    }
    
    public AutopilotInputsReader getReader() {
    	return this.reader;
    }
    
    public AutopilotOutputsWriter getWriter() {
    	return this.writer;
    }

    public void getOutput(java.io.DataInputStream inputStream, java.io.DataOutputStream outputStream) {
        AutopilotInputs input;
        try {
            input = getReader().read(inputStream);

            InputToOutput calc = new InputToOutput();
            if (this.previousInput==null) {
                AutopilotOutputs output= new AutopilotOutputs() {
                    public float getThrust() {return 0;}
                    public float getLeftWingInclination() {return 0;}
                    public float getRightWingInclination() {return 0;}
                    public float getHorStabInclination() {return 0;}
                    public float getVerStabInclination() {return 0;}};
            }
            else {
                AutopilotOutputs output = calc.calculate(input,imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns(), this);
            }
            try {
                getWriter().write(outputStream, output);
            } catch(IOException e) {
                e.printStackTrace();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.previousInput = input;
        this.previousOutput = output;

    }

    public AutopilotInputs getPreviousInput() {
        return this.previousInput;
    }

    public AutopilotOutputs getPreviousOutput() {
        return this.previousOutput;
    }

}
