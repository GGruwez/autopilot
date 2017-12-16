package interfaces;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;

class ImageRecognition {


	public static float[] FindTarget(byte[] image, int nbColumns, int nbRows ){
		HashSet<Float> hues = findDifferentColors(image, nbColumns, nbRows);
		if (hues.size() == 0){
			System.out.println("null");
			return null;
		}
		
		float[] bestCube = new float[] {0,0,0};
		Iterator<Float> iterator = hues.iterator();
		while (iterator.hasNext()){
			float hue = iterator.next();
			int hueMaxDifference = 1;
			float hueA = Math.max(hue - hueMaxDifference, 180 - Math.abs(hue-hueMaxDifference));
			float hueB = (hue + hueMaxDifference) % 180;
			float[] currentCube = findCubeInCertainColor(image, nbColumns, nbRows, hueA, hueB);
			if (currentCube != null && currentCube[2] > bestCube[2])
				bestCube = currentCube;
		}
//		for(float i = 0; i < hues.size(); i++){
//			float hue = hues.
//			cubes.add(findCubeInCertainColor(image, nbColumns, nbRows, 172, 8));
//		}
		System.out.println("[" + bestCube[0] + "," + bestCube[1] + "," + bestCube[2] + "]");
		return new float[]{bestCube[0], bestCube[1], bestCube[2]};
	}

	static HashSet<Float> findDifferentColors(byte[] image, int nbColumns, int nbRows){
		HashSet<Float> hues = new HashSet<Float>();
		for (int row=0; row<nbRows; row+=1) {
			for (int column =0; column<nbColumns ; column +=1){
				ArrayList<Integer> pixel = new ArrayList<Integer>();

				pixel.add(image[3*(row*nbColumns+column)+0] & 0xff) ;
				pixel.add(image[3*(row*nbColumns+column)+1] & 0xff);
				pixel.add(image[3*(row*nbColumns+column)+2] & 0xff);

				ArrayList<Float> HSVPixel = RBGToHSV(pixel);
				if((HSVPixel.get(1) >= 0.3) && (HSVPixel.get(2)>0.3)){ // Don't add black cubes
					hues.add(HSVPixel.get(0));
				}
			}
		}
		return hues;
	
	}
	
	
	
	static float[] findCubeInCertainColor(byte[] image, int nbColumns, int nbRows, float hueA, float hueB ){
		ArrayList<Float> positions = new ArrayList<Float>();
		for (int row=0; row<nbRows; row+=1) {
			for (int column =0; column<nbColumns ; column +=1){
				ArrayList<Integer> pixel = new ArrayList<Integer>();

				pixel.add(image[3*(row*nbColumns+column)+0] & 0xff) ;
				pixel.add(image[3*(row*nbColumns+column)+1] & 0xff);
				pixel.add(image[3*(row*nbColumns+column)+2] & 0xff);
				ArrayList<Float> HSVPixel = RBGToHSV(pixel);
				if (((hueA > hueB) ? ((HSVPixel.get(0) <= hueB) || (HSVPixel.get(0) >= hueA)) :	((HSVPixel.get(0) >= hueA && HSVPixel.get(0) <= hueB))) &&
						(HSVPixel.get(1) >= 0.3) && (HSVPixel.get(2)>0.3)){
					positions.add((float) (row*nbColumns+column));
				}
			}	
		}	
		
		if (positions.size()==0) {
			return null;
		}
		
		//ArrayList<Integer> filteredPositions = filterValues(positions,image);
		ArrayList<Float> filteredPositions = positions;
		
		
		int maxColumn = 0;
		int minColumn = nbColumns;
		int maxRow = 0;
		int minRow = nbRows;
		for (Float position :filteredPositions){
			

			int currentRow = (int) (position/nbColumns);
			int currentColumn = (int) (position%nbColumns);
			
			if (currentRow > maxRow)
				maxRow = currentRow;
			if (currentRow < minRow)
				minRow = currentRow;
			if (currentColumn > maxColumn)
				maxColumn = currentColumn;
			if (currentColumn < minColumn)
				minColumn = currentColumn;
		}
	
		
		int size = ((maxRow - minRow) == 0 ? 1 : (maxRow - minRow+1));
		float distance = (float) (0.7*200/size);

		float xVector = minColumn + (maxColumn - minColumn)/2 - nbColumns/2;
		float yVector = minRow + (maxRow - minRow)/2 - nbRows/2;

		return new float[]{xVector,-yVector, positions.size()};

		
	}
	
	static ArrayList<Float> RBGToHSV(ArrayList<Integer> pixel){
		float R = pixel.get(0)/255f;
		float G = pixel.get(1)/255f;
		float B = pixel.get(2)/255f;
		float Cmax = Max(R,G,B);
		float Cmin = Min(R,G,B);
		float delta = Cmax- Cmin;

		ArrayList<Float> HSVPixel = new ArrayList<Float>();
		
		if (delta == 0)
			HSVPixel.add(0f);
		else if (Cmax == R)
			HSVPixel.add((30*((G-B)/delta)%6));
		else if (Cmax == G)
			HSVPixel.add((30*((B-R)/delta)+2));
		else if (Cmax == B)
			HSVPixel.add((30*((R-G)/delta)+4));

		if (Cmax == 0){
			HSVPixel.add(0f);}
		else{
			HSVPixel.add(delta/Cmax);}		
		
		HSVPixel.add(Cmax);

		return HSVPixel;
	}
		
	
	static float Max(float R, float B, float G){
		if (R>B){
			if (R>G)
				return R;
			else
				return G;
			}
		else{
			if (B>G)
				return B;
			else
				return G;	
		}
	}
	
	static float Min(float R, float B, float G){
		
		if (R<B){
			if (R<G)
				return R;
			else
				return G;
			}
		else{
			if (B<G)
				return B;
			else
				return G;
			
		}
		
	}
			
	
	static ArrayList<Integer> filterValues(ArrayList<Integer> positions, byte[] image){

		
		ArrayList<ArrayList<Integer>> pos = new ArrayList<ArrayList<Integer>>();
		for (int i=0; i<6 ;i++){
			ArrayList<Integer> side = new ArrayList<Integer>();
			pos.add(side);
		}
		
		for (Integer position : positions){
			ArrayList<Integer> pixel = new ArrayList<Integer>();
			pixel.add(image[3*(position)+0] & 0xff) ;
			pixel.add(image[3*(position)+1] & 0xff);
			pixel.add(image[3*(position)+2] & 0xff);
			ArrayList<Float> HSVPixel = RBGToHSV(pixel);
			float value= HSVPixel.get(2);

			if (value <=0.225f){
				pos.get(0).add(position); //nY
			}else if (value <= 0.375f){
				pos.get(1).add(position); //nX
			}else if (value <= 0.59f){
				pos.get(2).add(position); //nZ
			}else if (value <= 0.775f){
				pos.get(3).add(position); //pZ
			}else if (value <= 0.925f){
				pos.get(4).add(position);//pX
			}else{
				pos.get(5).add(position);//pY
			}			
		}

		ArrayList<Integer> high = new ArrayList<Integer>();
		for (ArrayList<Integer> side :pos){

			if (side.size() > high.size())
				high = side;
		}
	
		return pos.get(3);
	}
}

