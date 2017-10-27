package src.autopilot;

import javax.swing.*;
import java.awt.*;

/**
 * Created by ndbae06 on 27/10/2017.
 */
public class UI extends JFrame {

    private float thrust;
    private String thrustText;
    private TextArea thrustTextArea;
    private float leftWingInclination;
    private String leftWingText;
    private TextArea leftWingTextArea;
    private float rightWingInclination;
    private String rightWingText;
    private TextArea rightWingTextArea;
    private float horStabInclination;
    private String horStabText;
    private TextField horstabTextArea;
    private float verStabInclination;
    private String verStabText;
    private TextField verstabTextArea;

    public UI() {
        super("UI");
        this.setVisible(true);
        this.setSize(new Dimension(200,200));
        this.setThrust(0);
        this.setHorStabInclination(0);
        this.setLeftWingInclination(0);
        this.setRightWingInclination(0);
        this.setRightWingInclination(0);
        this.thrustText = "Thrust:    ";
        this.thrustTextArea = new TextArea(this.getThrustText() + this.getThrust());
        this.add(this.getThrustTextArea());
        this.leftWingText = "LeftWingInclination:    ";
        this.leftWingTextArea = new TextArea(this.getLeftWingText()+this.getLeftWingInclination());
        this.getLeftWingTextArea().set
        this.add(this.getLeftWingTextArea());

    }

    //getters and setters
    private float getThrust() {
        return this.thrust;
    }
    private void setThrust(float t) {
        this.thrust = t;
    }
    private String getThrustText() {return this.thrustText;}
    public void setThrustText(String thrustText) {this.thrustText = thrustText;}
    public TextArea getThrustTextArea() {return thrustTextArea;}
    public void setThrustTextArea(TextArea thrustTextArea) {this.thrustTextArea = thrustTextArea;}
    public float getLeftWingInclination() {return leftWingInclination;}
    public void setLeftWingInclination(float leftWingInclination) {this.leftWingInclination = leftWingInclination;}
    public String getLeftWingText() {return leftWingText;}
    public void setLeftWingText(String leftWingText) {this.leftWingText = leftWingText;}
    public TextArea getLeftWingTextArea() {return leftWingTextArea;}
    public void setLeftWingTextArea(TextArea leftWingTextArea) {this.leftWingTextArea = leftWingTextArea;}
    public float getRightWingInclination() {return rightWingInclination;}
    public void setRightWingInclination(float rightWingInclination) {this.rightWingInclination = rightWingInclination;}
    public String getRightWingText() {return rightWingText;}
    public void setRightWingText(String rightWingText) {this.rightWingText = rightWingText;}
    public TextArea getRightWingTextArea() {return rightWingTextArea;}
    public void setRightWingTextArea(TextArea rightWingTextArea) {this.rightWingTextArea = rightWingTextArea;}
    public float getHorStabInclination() {return horStabInclination;}
    public void setHorStabInclination(float horStabInclination) {this.horStabInclination = horStabInclination;}
    public String getHorStabText() {return horStabText;}
    public void setHorStabText(String horStabText) {this.horStabText = horStabText;}
    public TextField getHorstabTextArea() {return horstabTextArea;}
    public void setHorstabTextArea(TextField horstabTextArea) {this.horstabTextArea = horstabTextArea;}
    public float getVerStabInclination() {return verStabInclination;}
    public void setVerStabInclination(float verStabInclination) {this.verStabInclination = verStabInclination;}
    public String getVerStabText() {return verStabText;}
    public void setVerStabText(String verStabText) {this.verStabText = verStabText;}
    public TextField getVerstabTextArea() {return verstabTextArea;}
    public void setVerstabTextArea(TextField verstabTextArea) {this.verstabTextArea = verstabTextArea;}



    public void updateData(autopilot.AutopilotOutputs data) {
        this.setThrust(data.getThrust());
    }

    private void simpleUpdate() {

    }
}
