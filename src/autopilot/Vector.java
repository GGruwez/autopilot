package autopilot;

import mygame.Vector;

public class Vector {

	private final float x;
	private final float y;
	private final float z;

	public Vector(float x, float y, float z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	
	public float getX(){
		return this.x;
	}
	
	public float getY(){
		return this.y;
		
	}
	
	public float getZ(){
		return this.z;
	}
	


	public Vector add(Vector other){
		return new Vector(this.getX()+ other.getX(), this.getY() + other.getY(), this.getZ() + other.getZ());
	}
	
	public Vector subtract(Vector other){
		return new Vector(this.getX()- other.getX(), this.getY() - other.getY(), this.getZ() - other.getZ());

	}
	
	public float euclideanLength(){
		return (float) Math.sqrt(this.dotProduct(this));
	}
	
	/**
	 * returnt de hoek tussen twee vectoren in radialen!
	 */
	public float angleBetween(Vector other){
		return (float) Math.acos(this.dotProduct(other)/(this.euclideanLength()*other.euclideanLength()));
	}
	
	public void printVector(String name){
		System.out.println(name + this.getX() + " " + this.getY() + " " + this.getZ());
	}
	/**
	 * 
	 * @param heading - heading van de aircraft tov wereld
	 * @param pitch - pitch van de aircraft tov wereld
	 * @param roll - roll van de aircraft tov wereld
	 * @return
	 */

	public Vector transform(float heading, float pitch, float roll ){
	
		
		double newX = this.x*(Math.cos(heading)*Math.cos(roll)-Math.sin(heading)*Math.sin(pitch)*Math.sin(roll))+this.y*(Math.cos(heading)*Math.sin(roll)+Math.cos(roll)*Math.sin(heading)*Math.sin(pitch))+this.z*(-Math.cos(pitch)*Math.sin(heading));
		float X = (float)newX;
		
		double newY = this.x*(-Math.cos(pitch)*Math.sin(roll))+this.y*(Math.cos(pitch)*Math.cos(roll))+this.z*(Math.sin(pitch));
		float Y = (float)newY;
		
		double newZ = this.x*(Math.cos(roll)*Math.sin(heading)+Math.cos(heading)*Math.sin(pitch)*Math.sin(roll))+this.y*(Math.sin(heading)*Math.sin(roll)-Math.cos(heading)*Math.cos(roll)*Math.sin(pitch))+this.z*(Math.cos(heading)*Math.cos(pitch));
		float Z = (float)newZ;
		
		return new Vector(X,Y,Z);
	}
	
	//inverse matrix gewoon getransponeerde van normale matrix
	public Vector inverseTransform(float heading, float pitch, float roll){
		double newX = this.x*(Math.cos(heading)*Math.cos(roll)-Math.sin(heading)*Math.sin(pitch)*Math.sin(roll)) + this.y*(-Math.cos(pitch)*Math.sin(roll)) + this.z*(Math.cos(roll)*Math.sin(heading)+Math.cos(heading)*Math.sin(pitch)*Math.sin(roll));
		float X = (float)newX;
		
		double newY = this.x*(Math.cos(heading)*Math.sin(roll)+Math.cos(roll)*Math.sin(heading)*Math.sin(pitch)) + this.y*(Math.cos(pitch)*Math.cos(roll)) + this.z*(Math.sin(heading)*Math.sin(roll)-Math.cos(heading)*Math.cos(roll)*Math.sin(pitch));
		float Y = (float)newY;
		
		double newZ = this.x*(-Math.cos(pitch)*Math.sin(heading)) + this.y*(Math.sin(pitch)) + this.z*(Math.cos(heading)*Math.cos(pitch));
		float Z = (float)newZ;
		
		return new Vector(X,Y,Z);
				
	}
	
	/**
	 * A.B
	 * @param other
	 * @return
	 */
	public float dotProduct(Vector other){
		return this.x*other.x + this.y*other.y + this.z*other.z;
	}
	
	/**
	 * AxB
	 * @param other
	 * @return
	 */
	public Vector crossProduct(Vector other){
		return new Vector(this.y*other.z - other.y*this.z,
				-this.x*other.z+other.x*this.z, this.x*other.y - other.x*this.y);
	}
	
	
	public Vector constantProduct(float constant){
		return new Vector(this.x*constant, this.y*constant, this.z*constant);
	}
        
	@Override
    public Vector clone(){
        return new Vector(this.getX(), this.getY(), this.getZ());
    }  
}	