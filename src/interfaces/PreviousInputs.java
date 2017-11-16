package interfaces;


public class PreviousInputs {
	byte[] image;
	float x;
	float y;
	float z;
	float heading;
	float roll;
	float pitch;
	float elapsedTime;
	 
	 PreviousInputs (AutopilotInputs input){
		 image = input.getImage();
		 x = input.getX();
		 y = input.getY();
		 z = input.getZ();
		 roll = input.getRoll();
		 heading = input.getHeading();
		 pitch = input.getPitch();
		 elapsedTime = input.getElapsedTime();
	 }
	 
	 
	 
	 
     public byte[] getImage() {
         return image; // TODO: get image
     }

    
     public float getX() {
         return x;
     }

   
     public float getY() {
         return y;
     }

  
     public float getZ() {
         return z;
     }

 
     public float getHeading() {
         return heading;
     }

 
     public float getPitch() {
         return pitch;
     }

   
     public float getRoll() {
         return roll;
     }

 
     public float getElapsedTime() {

         return elapsedTime;
     }
}
