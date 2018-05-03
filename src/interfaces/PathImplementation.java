package interfaces;

import java.util.ArrayList;

public class PathImplementation implements Path {
	
	public PathImplementation(ArrayList<Vector> newPath) {
		float[] x = new float[newPath.size()];
		float[] y = new float[newPath.size()];
		float[] z = new float[newPath.size()];
		int i = 0;
		for (Vector Path : newPath) {
			x[i] = Path.getX();
			y[i] = Path.getY();
			z[i] = Path.getZ();
			i++;
		}
		
		this.X = x;
		this.Y = y;
		this.Z = z;
		System.out.println(x);
		System.out.println(y);
		System.out.println(z);
	}

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
	

	public float[] X = new float[]{};
	public float[] Y = new float[]{};
	public float[] Z = new float[]{};
	
	
}
