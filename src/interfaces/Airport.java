package interfaces;

public class Airport {
	
	public Airport(float X, float Z, float toRunwayX, float toRunwayZ) {
		this.centerX = X;
		this.centerZ = Z;
		this.centerToRunway0X = toRunwayX;
		this.centerToRunway0Z = toRunwayZ;
	}
	
	float getCenterX() {
		return this.centerX;
	}
	
	float getCenterZ() {
		return this.centerZ;
	}
	
	float getCenterToRunway0X() {
		return this.centerToRunway0X;
	}
	
	float getCenterToRunway0Z() {
		return this.centerToRunway0Z;
	}
	
	private float centerX;
	private float centerZ;
	private float centerToRunway0X;
	private float centerToRunway0Z;
	
}
