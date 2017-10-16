
package autopilot;
import  p_en_o_cw_2017.AutopilotOutputs;
import  p_en_o_cw_2017.AutopilotInputs;
public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] imageVector, int nbColumns, int nbRows) {
        float x = imageVector[0];
        float y = imageVector[1];
        float leftWingInclination;
        float rightWingInclination;
        float horStabInclination;
        if (imageVector[0]!=0 || imageVector[1]!=0) {
            //if (input.getPitch() == 0 && input.getRoll() == 0) {
                //turning left or right
                    float angle = x/(nbColumns/2)*45;
                    leftWingInclination = angle;
                    rightWingInclination = -angle;
                //up or down
                    angle = y/(nbRows/2)*45;
                    horStabInclination = angle;
            //}
        }
        else {
            leftWingInclination = 0;
            rightWingInclination = 0;
            horStabInclination = 0;
        }
        return new AutopilotOutputs() {
            public float getThrust() { return 0; }
            public float getLeftWingInclination() { return leftWingInclination; }
            public float getRightWingInclination() { return rightWingInclination; }
            public float getHorStabInclination() { return horStabInclination; }
            public float getVerStabInclination() { return 0; }
        };
    }


}