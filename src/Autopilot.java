import autopilot.p_en_o_cw_2017.autopilot_src_generated.*;


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

    public java.io.DataOutputStream getOutput(java.io.DataInputStream inputStream) {
        AutopilotInputs input = new AutopilotInputs() {
            public byte[] getImage() {return new byte[0];}
            public float getX() {return 0;}
            public float getY() {return 0;}
            public float getZ() {return 0;}
            public float getHeading() {return 0;}
            public float getPitch() {return 0;}
            public float getRoll() {return 0;}
            public float getElapsedTime() {return 0;}};
        try {
            input = reader.read(inputStream);
        } catch (IOException e) {
            e.printStackTrace();
        }
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

    }

    public java.io.DataOutputStream getOutputFirstTime(java.io.DataInputStream inputStream) {

    }


}
