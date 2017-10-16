package autopilot;
import p_en_o_cw_2017.AutopilotInputsReader ;
import p_en_o_cw_2017.AutopilotOutputsWriter ;
import p_en_o_cw_2017.AutopilotConfigReader;
import p_en_o_cw_2017.AutopilotInputs;
import p_en_o_cw_2017.AutopilotOutputs;
import p_en_o_cw_2017.AutopilotConfig;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;

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

        try {
            AutopilotInputs input = reader.read(inputStream);
            InputToOutput calc = new InputToOutput();
            AutopilotOutputs output = calc.calculate(input,imageRecognition.FindTarget(input.getImage(), config.getNbColumns(),config.getNbRows()), config.getNbRows(), config.getNbColumns());
            DataOutputStream outputStream = new DataOutputStream(new OutputStream() {
                @Override
                public void write(int b) throws IOException {

                }
            });
            writer.write(outputStream, output);
            return outputStream;

        } catch (IOException e) {
            e.printStackTrace();
        }


    }


}
