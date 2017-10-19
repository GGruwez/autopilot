import p_en_o_cw_2017.*;

public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] imageVector, int nbColumns, int nbRows) {
        float x = imageVector[0];
        float y = imageVector[1];
        float leftWingInclination = 0;
        float rightWingInclination = 0;
        float horStabInclination = 0;
        

        return new AutopilotOutputs() {
            public float getThrust() { return 0; }
            public float getLeftWingInclination() { return leftWingInclination; }
            public float getRightWingInclination() { return rightWingInclination; }
            public float getHorStabInclination() { return horStabInclination; }
            public float getVerStabInclination() { return 0; }
        };
    }


}