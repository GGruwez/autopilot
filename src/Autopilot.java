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
    
    public AutopilotInputsReader getReader() {
    	return this.reader;
    }
    
    public AutopilotOutputsWriter getWriter() {
    	return this.writer;
    }

    public void getOutput(java.io.DataInputStream inputStream, java.io.DataOutputStream outputStream) {
        AutopilotInputs input = };
        try {
            input = getReader().read(inputStream);
        } catch (IOException e) {
            e.printStackTrace();
        }
        InputToOutput calc = new InputToOutput();
        AutopilotOutputs output = calc.calculate(input,imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns());
        try {
            getWriter().write(outputStream, output);
        } catch(IOException e) {
            e.printStackTrace();
        }

    }


}
