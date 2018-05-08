package interfaces;

public class Airport {
	
	public Airport(float X, float Z, float toRunwayX, float toRunwayZ) {
		this.centerX = X;
		this.centerZ = Z;
		float norm = (float) Math.sqrt(toRunwayZ*toRunwayZ + toRunwayX*toRunwayX);
		this.centerToRunway0X = toRunwayX/norm;
		this.centerToRunway0Z = toRunwayZ/norm;
	}
	
	public float getCenterX() {
		return this.centerX;
	}
	
	public float getCenterZ() {
		return this.centerZ;
	}
	
	public float getCenterToRunway0X() {
		return this.centerToRunway0X;
	}
	
	public float getCenterToRunway0Z() {
		return this.centerToRunway0Z;
	}
	
	public float getDistanceToAirport(Airport airport) {
		return (float) Math.sqrt(
    			Math.pow(this.getCenterX()-airport.getCenterX(),2) +
    			Math.pow(this.getCenterZ()-airport.getCenterZ(),2)
    			);
	}
	
	public boolean hasDroneAt(int gate) {
		if (gate == 0) {
			return (drone0 != null);
		}
		return drone1 != null;
	}
	
	public AutopilotImplementation getDroneAt(int gate) {
		if (gate == 0) {
			return drone0;
		}
		return drone1;
	}
	
	public void setDroneAt(int gate, AutopilotImplementation drone) {
		if (gate == 0) {
			drone0 = drone;
		}
		else {
			drone1 = drone;
		}
	}
	
	private float centerX;
	private float centerZ;
	private float centerToRunway0X;
	private float centerToRunway0Z;
	private AutopilotImplementation drone0 = null;
	private AutopilotImplementation drone1 = null;
	
}
