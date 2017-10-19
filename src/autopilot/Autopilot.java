package autopilot;

import p_en_o_cw_2017.*;

import java.io.*;

/**
 * Created by ndbae06 on 16/10/2017.
 */
public class Autopilot {
    ImageRecognition imageRecognition;
    InputToOutput inputToOutput;
    AutopilotConfigReader configReader;
    AutopilotConfig config;
    AutopilotInputs previousInput;
    AutopilotOutputs previousOutput;

    public Autopilot(DataInputStream configstream) {
        this.imageRecognition = new ImageRecognition();
        this.inputToOutput = new InputToOutput();
    }

    public void fillStreamWithOutput(java.io.DataInputStream inputStream, java.io.DataOutputStream outputStream) {
        AutopilotInputs input;
        AutopilotOutputs output;
        try {
            input = AutopilotInputsReader.read(inputStream);

            InputToOutput calc = new InputToOutput();
            if (this.previousInput==null) {
                output = new AutopilotOutputs();
            }
            else {
                output = calc.calculate(input,imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns(), this);
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

}
