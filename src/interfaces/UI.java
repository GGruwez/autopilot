package interfaces;

import javax.swing.*;
import java.awt.*;

class UI extends JFrame {

    private static String thrustText = "Thrust:    ";
    private JLabel thrustJLabel = new JLabel();
    private static String leftWingText = "LeftWingInclination:    ";
    private JLabel leftWingJLabel = new JLabel();
    private static String rightWingText = "RightWingInclination:    ";
    private JLabel rightWingJLabel = new JLabel();
    private static String horStabText = "HorStabInclination:    ";
    private JLabel horstabJLabel = new JLabel();
    private static String verStabText = "VerStabInclination:    ";
    private JLabel verstabJLabel = new JLabel();
    private static String statusText = "Status:    ";
    private JLabel statusJLabel = new JLabel();

    private AutopilotOutputsImplementation autopilotOutput;
    private AutopilotModuleImplementation module;
    private AutopilotImplementation autopilot;

    UI() {
        super("UI");
        this.setVisible(true);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setSize(new Dimension(200,200));
        this.setLocation(10,10);
        this.setLayout(new BoxLayout(this.getContentPane(),BoxLayout.Y_AXIS));
        this.add(this.getThrustJLabel());
        this.add(this.getLeftWingJLabel());
        this.add(this.getRightWingJLabel());
        this.add(this.getVerstabJLabel());
        this.add(this.getHorstabJLabel());
    }

    //getters and setters
    private String getThrustText() {return thrustText;}
    private JLabel getThrustJLabel() {return thrustJLabel;}
    private String getLeftWingText() {return leftWingText;}
    private JLabel getLeftWingJLabel() {return leftWingJLabel;}
    private String getRightWingText() {return rightWingText;}
    private JLabel getRightWingJLabel() {return rightWingJLabel;}
    private String getHorStabText() {return horStabText;}
    private JLabel getHorstabJLabel() {return horstabJLabel;}
    private String getVerStabText() {return verStabText;}
    private JLabel getVerstabJLabel() {return verstabJLabel;}
    private String getStatusText() {return statusText;}
    private JLabel getStatusJLabel() {return statusJLabel;}
    private AutopilotOutputsImplementation getAutopilotOutput(){
        return this.autopilotOutput;
    }
    private AutopilotModuleImplementation getModule() {
    	return this.module;
    }
    private AutopilotImplementation getAutopilot() {
    	return this.autopilot;
    }


    void updateData(AutopilotOutputsImplementation data, AutopilotModuleImplementation module, AutopilotImplementation autopilot) {
        this.autopilotOutput = data;
        this.module = module;
        this.autopilot = autopilot;
        this.simpleUpdate();
    }

    private void simpleUpdate() {
        this.getLeftWingJLabel().setText(this.getLeftWingText()+this.getAutopilotOutput().getLeftWingInclination());
        this.getRightWingJLabel().setText(this.getRightWingText()+this.getAutopilotOutput().getRightWingInclination());
        this.getThrustJLabel().setText((this.getThrustText()+this.getAutopilotOutput().getThrust()));
        this.getHorstabJLabel().setText(this.getHorStabText()+this.getAutopilotOutput().getHorStabInclination());
        this.getVerstabJLabel().setText(this.getVerStabText()+this.getAutopilotOutput().getVerStabInclination());
        
        String status = "idle";
        for (Job job: getModule().getJobs()) {
        	if (job == this.getAutopilot().getCurrentJob()) {
        		if (this.getAutopilot().getDrone().getAirport() == job.getAirportFrom()) {
        			status = "in transit";
        		}
        		else if (this.getAutopilot().getDrone().getAirport() == job.getAirportTo()) {
        			status = "delivering";
        		}
        	}
        }
        
        this.getStatusJLabel().setText(this.getStatusText() + status);
    }
}
