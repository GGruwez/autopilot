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
        AutopilotOutputs output = calc.calculate(
                input,
                imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()),
                config.getNbRows(),
                config.getNbColumns());
        DataOutputStream outputStream = new DataOutputStream(new ByteArrayOutputStream());
        try {
            writer.write(outputStream, output);
        } catch(IOException e) {
            e.printStackTrace();
        }

        return outputStream;

    }


}
