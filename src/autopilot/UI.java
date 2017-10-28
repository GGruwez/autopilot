package autopilot;

import javax.swing.*;
import java.awt.*;

/**
 * Created by ndbae06 on 27/10/2017.
 */
public class UI extends JFrame {

    private float thrust;
    private String thrustText;
    private JLabel thrustJLabel;
    private float leftWingInclination;
    private String leftWingText;
    private JLabel leftWingJLabel;
    private float rightWingInclination;
    private String rightWingText;
    private JLabel rightWingJLabel;
    private float horStabInclination;
    private String horStabText;
    private JLabel horstabJLabel;
    private float verStabInclination;
    private String verStabText;
    private JLabel verstabJLabel;

    public UI() {
        super("UI");
        this.setVisible(true);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setSize(new Dimension(200,200));
        this.setLocation(1120,250);
        this.setLayout(new BoxLayout(this.getContentPane(),BoxLayout.Y_AXIS));
        this.setThrust(0);
        this.setHorStabInclination(0);
        this.setLeftWingInclination(0);
        this.setRightWingInclination(0);
        this.setRightWingInclination(0);
        this.thrustText = "Thrust:    ";
        this.thrustJLabel = new JLabel(this.getThrustText() + this.getThrust());
        this.add(this.getThrustJLabel());
        this.leftWingText = "LeftWingInclination:    ";
        this.leftWingJLabel = new JLabel(this.getLeftWingText()+this.getLeftWingInclination());
        this.add(this.getLeftWingJLabel());
        this.rightWingText = "RightWingInclination:    ";
        this.rightWingJLabel = new JLabel(this.getRightWingText()+this.getRightWingInclination());
        this.add(this.getRightWingJLabel());
        this.verStabText = "VerStabInclination:    ";
        this.verstabJLabel = new JLabel(this.getVerStabText() + this.getVerStabInclination());
        this.add(this.getVerstabJLabel());
        this.horStabText = "HorStabInclination:    ";
        this.horstabJLabel = new JLabel(this.getHorStabText() + this.getHorStabInclination());
        this.add(this.getHorstabJLabel());
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
    public JLabel getThrustJLabel() {return thrustJLabel;}
    public void setThrustJLabel(JLabel thrustJLabel) {this.thrustJLabel = thrustJLabel;}
    public float getLeftWingInclination() {return leftWingInclination;}
    public void setLeftWingInclination(float leftWingInclination) {this.leftWingInclination = leftWingInclination;}
    public String getLeftWingText() {return leftWingText;}
    public void setLeftWingText(String leftWingText) {this.leftWingText = leftWingText;}
    public JLabel getLeftWingJLabel() {return leftWingJLabel;}
    public void setLeftWingJLabel(JLabel leftWingJLabel) {this.leftWingJLabel = leftWingJLabel;}
    public float getRightWingInclination() {return rightWingInclination;}
    public void setRightWingInclination(float rightWingInclination) {this.rightWingInclination = rightWingInclination;}
    public String getRightWingText() {return rightWingText;}
    public void setRightWingText(String rightWingText) {this.rightWingText = rightWingText;}
    public JLabel getRightWingJLabel() {return rightWingJLabel;}
    public void setRightWingJLabel(JLabel rightWingJLabel) {this.rightWingJLabel = rightWingJLabel;}
    public float getHorStabInclination() {return horStabInclination;}
    public void setHorStabInclination(float horStabInclination) {this.horStabInclination = horStabInclination;}
    public String getHorStabText() {return horStabText;}
    public void setHorStabText(String horStabText) {this.horStabText = horStabText;}
    public JLabel getHorstabJLabel() {return horstabJLabel;}
    public void setHorstabJLabel(JLabel horstabJLabel) {this.horstabJLabel = horstabJLabel;}
    public float getVerStabInclination() {return verStabInclination;}
    public void setVerStabInclination(float verStabInclination) {this.verStabInclination = verStabInclination;}
    public String getVerStabText() {return verStabText;}
    public void setVerStabText(String verStabText) {this.verStabText = verStabText;}
    public JLabel getVerstabJLabel() {return verstabJLabel;}
    public void setVerstabJLabel(JLabel verstabJLabel) {this.verstabJLabel = verstabJLabel;}



    public void updateData(AutopilotOutputs data) {
        this.setThrust(data.getThrust());
        this.setLeftWingInclination(data.getLeftWingInclination());
        this.setRightWingInclination(data.getRightWingInclination());
        this.setHorStabInclination(data.getHorStabInclination());
        this.setVerStabInclination(data.getVerStabInclination());
        this.simpleUpdate();
    }

    private void simpleUpdate() {
        this.getLeftWingJLabel().setText(this.getLeftWingText()+this.getLeftWingInclination());
        this.getRightWingJLabel().setText(this.getRightWingText()+this.getRightWingInclination());
        this.getThrustJLabel().setText((this.getThrustText()+this.getThrust()));
        this.getHorstabJLabel().setText(this.getHorStabText()+this.getHorStabInclination());
        this.getVerstabJLabel().setText(this.getVerStabText()+this.getVerStabInclination());
    }
}
