package interfaces;

public class Job {
	
	public Job(Airport from, int gateFrom, Airport to, int gateTo) {
		this.from = from;
		this.to = to;
		this.gateFrom = gateFrom;
		this.gateTo = gateTo;
	}
	
	public Airport getAirportFrom() {
		return this.from;
	}
	
	public Airport getAirportTo() {
		return this.to;
	}
	
	public Autopilot getDrone() {
		return this.drone;
	}
	
	public void setDrone(Autopilot drone) {
		if (! this.hasDrone()) {
			this.drone = drone;
		}
	}
	
	public boolean hasDrone() {
		return ! (this.getDrone() == null);
	}
	
	public int getGateFrom() {
		return this.gateFrom;
	}
	
	public int getGateTo() {
		return this.gateTo;
	}
  	
	private Airport from;
	private Airport to;
	private Autopilot drone = null;
	private int gateFrom;
	private int gateTo;
	
}
