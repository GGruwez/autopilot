package autopilot;

/**
 * Created by ndbae06 on 16/10/2017.
 */
public class Autopilot {
    ImageRecognition imageRecognition;
    InputToOutput inputToOutput;

    public Autopilot() {
        this.imageRecognition = new ImageRecognition();
        this.inputToOutput = new InputToOutput();
    }
    


}
