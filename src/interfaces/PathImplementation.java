package interfaces;

public class PathImplementation implements Path {

	@Override
	public float[] getX() {
		return this.X;
	}

	@Override
	public float[] getY() {
		return this.Y;
	}

	@Override
	public float[] getZ() {
		return this.Z;
	}
	
	public float[] X = new float[]{100,-200};
	public float[] Y = new float[]{40,60};
	public float[] Z = new float[]{-800,40};
}
