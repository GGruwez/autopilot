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

            AutopilotOutputs output = calc.calculate(
                    input,
                    imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()),
                    config.getNbRows(),
                    config.getNbColumns());

            try {
                getWriter().write(outputStream, output);
            } catch(IOException e) {
                e.printStackTrace();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
<<<<<<< HEAD
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
            AutopilotOutputs output = calc.calculate(input,imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns(), this.previousInput, this);
        }
        DataOutputStream outputStream = new DataOutputStream(new ByteArrayOutputStream());
        try {
            writer.write(outputStream, output);
        } catch(IOException e) {
            e.printStackTrace();
        }
        this.previousInput = input;
        return outputStream;
=======

>>>>>>> 4a9f7758de3cd800b3c95df128aec3669ebf7849

    }

    public java.io.DataOutputStream getOutputFirstTime(java.io.DataInputStream inputStream) {

    }


}
