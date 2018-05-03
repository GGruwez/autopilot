package interfaces;

public class Vector {
    
        public final static Vector NULL = new Vector(0, 0, 0);

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
        
        public Vector checkAndNeglect(float rejectValue){
            float newX = this.getX();
            float newY = this.getY();
            float newZ = this.getZ();
            if (Math.abs(newX)< rejectValue){
                newX = 0;
            }
            if (Math.abs(newY) < rejectValue){
                newY = 0;
            }
            if (Math.abs(newZ) < rejectValue){
                newZ = 0;
            }
            return new Vector(newX,newY,newZ);
            
        }
        
        @Override
        public String toString(){
            return String.format("(%.0f, %.0f, %.0f)", this.getX(), this.getY(), this.getZ());
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
	
        public double calculateDistance(Vector other) {
            float x1 = other.getX();
            float y1 = other.getY();
            float z1 = other.getZ();
            return Math.sqrt(Math.pow(x-x1, 2)+Math.pow(y-y1, 2)+Math.pow(z-z1, 2));
        }
        
	/**
	 * returnt de hoek tussen twee vectoren in radialen!
	 */
	public float angleBetween(Vector other){
		return (float) Math.acos(this.dotProduct(other)/(this.euclideanLength()*other.euclideanLength()));
	}
	

	/**
	 * AIRCRAFT -> WERELD
	 * @param heading - heading van de aircraft tov wereld rond y-as
	 * @param pitch - pitch van de aircraft tov wereld rond x-as
	 * @param roll - roll van de aircraft tov wereld rond z-as
	 * @return
	 */
	public Vector transform(float heading, float pitch, float roll ){
		double newX = this.x*(Math.cos(heading)*Math.cos(roll)+Math.sin(heading)*Math.sin(pitch)*Math.sin(roll))+this.y*(-Math.cos(heading)*Math.sin(roll)+Math.cos(roll)*Math.sin(heading)*Math.sin(pitch))+this.z*(Math.cos(pitch)*Math.sin(heading));
		float X = (float)newX;
		
		double newY = this.x*(Math.cos(pitch)*Math.sin(roll))+this.y*(Math.cos(pitch)*Math.cos(roll))+this.z*(-Math.sin(pitch));
		float Y = (float)newY;
		
		double newZ = this.x*(-Math.cos(roll)*Math.sin(heading)+Math.cos(heading)*Math.sin(pitch)*Math.sin(roll))+this.y*(Math.sin(heading)*Math.sin(roll)+Math.cos(heading)*Math.cos(roll)*Math.sin(pitch))+this.z*(Math.cos(heading)*Math.cos(pitch));
		float Z = (float)newZ;
		
		return new Vector(X,Y,Z);
	}
	
	/**
	 * WERELD --> AIRCRAFT
	 */
	//inverse matrix gewoon getransponeerde van normale matrix
	public Vector inverseTransform(float heading, float pitch, float roll){
		double newX = this.x*(Math.cos(heading)*Math.cos(roll)+Math.sin(heading)*Math.sin(pitch)*Math.sin(roll)) + this.y*(Math.cos(pitch)*Math.sin(roll)) + this.z*(-Math.cos(roll)*Math.sin(heading)+Math.cos(heading)*Math.sin(pitch)*Math.sin(roll));
		float X = (float)newX;
		
		double newY = this.x*(-Math.cos(heading)*Math.sin(roll)+Math.cos(roll)*Math.sin(heading)*Math.sin(pitch)) + this.y*(Math.cos(pitch)*Math.cos(roll)) + this.z*(Math.sin(heading)*Math.sin(roll)+Math.cos(heading)*Math.cos(roll)*Math.sin(pitch));
		float Y = (float)newY;
		
		double newZ = this.x*(Math.cos(pitch)*Math.sin(heading)) + this.y*(-Math.sin(pitch)) + this.z*(Math.cos(heading)*Math.cos(pitch));
		float Z = (float)newZ;
		
		return new Vector(X,Y,Z);
				
	}
	
        public void printVector(String name){
        System.out.println(name + this.getX() + " " + this.getY() + " " + this.getZ());
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
        
	public Vector applyInertiaTensor(Vector InertiaTensor){
		return new Vector(this.getX()*InertiaTensor.getX(), this.getY()*InertiaTensor.getY(), this.getZ()*InertiaTensor.getZ());
	}
	
	@Override
    public Vector clone(){
        return new Vector(this.getX(), this.getY(), this.getZ());
    }  
	
    public static Vector getInertiaTensor(AutopilotConfig config){
	    //elementen vd matrix berekenen - alles behalve elementen op de diagonaal zijn 0

		    double Ixx1 = Math.pow(config.getTailSize()
		            ,2)*config.getTailMass() + Math.pow(getEnginePlace(config),2)*config.getEngineMass();
		    float Ixx = (float)Ixx1;
		
		
		    double Iyy1 = 2*Math.pow(config.getWingX()
		            , 2)*config.getWingMass() + Math.pow(config.getTailSize(),2)*config.getTailMass() + Math.pow(getEnginePlace(config),2)*config.getEngineMass();
		
		    float Iyy = (float)Iyy1;
		
		    double Izz1 = 2*Math.pow(config.getWingX(), 2)*config.getWingMass();
		    float Izz = (float)Izz1;
		
		    return new Vector(Ixx,Iyy,Izz);
    }
	 
    public static float getEnginePlace(AutopilotConfig config){
	    return config.getTailSize()*(-config.getTailMass()/config.getEngineMass());
	}
    
    public boolean equals(Vector other){
    	return (this.getX() == other.getX() && this.getY() == other.getY() && this.getZ() == other.getZ());
    }
}	
	
		