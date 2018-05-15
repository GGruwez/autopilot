package interfaces;

import com.sun.xml.internal.bind.v2.TODO;

import java.util.ArrayList;

public class Job {
	
	public Job(Airport from, int gateFrom, Airport to, int gateTo) {
		this.from = from;
		this.to = to;
		this.gateFrom = gateFrom;
		this.gateTo = gateTo;
		this.path = calculatePath();
	}
	
	public Airport getAirportFrom() {
		return this.from;
	}
	
	public Airport getAirportTo() {
		return this.to;
	}
	
	public AutopilotImplementation getDrone() {
		return this.drone;
	}
	
	public PathImplementation getPath(){
		return this.path;
	}
	
	public void setDrone(AutopilotImplementation drone) {
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
	
	public PathImplementation calculatePath(){
		ArrayList<Vector> path = new ArrayList<Vector>();
		float takeoffLenght = 380;
		float turningRadius = 900;
		Vector startingPoint = new Vector(from.getCenterX()+takeoffLenght*from.getCenterToRunway0X(),20,from.getCenterZ()+ takeoffLenght*from.getCenterToRunway0Z());
		Vector centerLStart = new Vector((float)(startingPoint.getX() + turningRadius*from.getCenterToRunway0Z()),0,(float)(startingPoint.getZ() - turningRadius*from.getCenterToRunway0X()));
		Vector centerRStart = new Vector((float)(startingPoint.getX() - turningRadius*from.getCenterToRunway0Z()),0,(float)(startingPoint.getZ() + turningRadius*from.getCenterToRunway0X()));
		Vector endPoint = new Vector(to.getCenterX()-takeoffLenght*to.getCenterToRunway0X(),20,to.getCenterZ()- takeoffLenght*to.getCenterToRunway0Z());
		Vector centerLEnd = new Vector((float)(endPoint.getX() + turningRadius*to.getCenterToRunway0Z()),0,(float)(endPoint.getZ() - turningRadius*to.getCenterToRunway0X()));
		Vector centerREnd = new Vector((float)(endPoint.getX() - turningRadius*to.getCenterToRunway0Z()),0,(float)(endPoint.getZ() + turningRadius*to.getCenterToRunway0X()));
		
		if(centerRStart.calculateDistance(endPoint) < centerLStart.calculateDistance(endPoint)) {
			//takeRightStartCircle
			if(centerRStart.calculateDistance(centerREnd) < centerRStart.calculateDistance(centerLEnd)) {
				//takeRightEndCircle

				System.out.println("R-R");

				float x1 = centerRStart.getX();
				float y1 = centerRStart.getZ();
				float x2 = centerREnd.getX();
				float y2 = centerREnd.getZ();
				float r = turningRadius;
				Vector offset = centerREnd.subtract(centerRStart);
				Vector toT = offset.constantProduct(r/offset.euclideanLength());
				toT = new Vector(-toT.getZ(),0,toT.getX());
				Vector t1 = centerRStart.add(toT);
				Vector t2 = centerRStart.subtract(toT);



				centerRStart.printVector("start:  ");
				centerREnd.printVector("end:  ");
				t1.printVector("t1:  ");
				t2.printVector("t2:  ");



				Vector r1 = t1.add(offset);
				Vector r2 = t2.add(offset);
				r1.printVector("r1:  ");
				r2.printVector("r2:  ");


				//get path on first circle
//todo add support for turned airports
				Vector a1 = t2.subtract(centerRStart);
				Vector a2 = startingPoint.subtract(centerRStart);
				float angle = a2.angleBetween(a1);
				float arclenght = (float) (turningRadius*angle/(2*Math.PI));
				float nbPathentry = arclenght/20;
				float stepAngle = angle/nbPathentry;

				Vector nzAxis = new Vector(0,0,-1);
				Vector Airportaxis = new Vector(from.getCenterToRunway0X(),0,from.getCenterToRunway0Z());
				float bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() < 0){
					bias -= Math.PI;
				}
				System.out.println(bias);
				float signx = -1;
				float signz = -1;

				for (int i=0;i<nbPathentry;i++) {
					float x = (float) (centerRStart.getX() + signx * Math.cos(bias + stepAngle * i)*turningRadius);
					float y = (float) (centerRStart.getZ() + signz * Math.sin(bias + stepAngle * i)*turningRadius);
					Vector pathEntry = new Vector(x,20,y);
					path.add(pathEntry);

				}
				float x = (float) (centerRStart.getX() + signx * Math.cos(bias + angle)*turningRadius);
				float y = (float) (centerRStart.getZ() + signz * Math.sin(bias + angle)*turningRadius);
				Vector pathEntry = new Vector(x,20,y);
				path.add(pathEntry);

				// get path on tangent

				Vector tangent = r2.subtract(t2);
				nbPathentry = tangent.euclideanLength()/20;
				for (int i= 1; i <nbPathentry;i++) {
					float partOfTangent = i/nbPathentry;
					pathEntry = t2.add(tangent.constantProduct(partOfTangent));
					pathEntry = new Vector(pathEntry.getX(),20,pathEntry.getZ());
					path.add(pathEntry);
				}


				// get path on second circle

				a1 = r2.subtract(centerREnd);
				a2 = endPoint.subtract(centerREnd);
				angle = a2.angleBetween(a1);

				arclenght = (float) (turningRadius*angle/(2*Math.PI));
				nbPathentry = arclenght/20;
				stepAngle = angle/nbPathentry;
				nzAxis = new Vector(0,0,-1);
				Airportaxis = new Vector(to.getCenterToRunway0X(),0,to.getCenterToRunway0Z());
				bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() < 0) {
					bias -= Math.PI;
				}
				System.out.println(bias);
				signx = -1;
				signz = 1;
				for (int i= 1; i <nbPathentry;i++) {
					x = (float) (centerREnd.getX() + signx * Math.cos(bias + angle - stepAngle * i)*turningRadius);
					y = (float) (centerREnd.getZ() + signz * Math.sin(bias + angle - stepAngle * i)*turningRadius);
					pathEntry = new Vector(x,20,y);
					path.add(pathEntry);
				}

				path.add(endPoint);


			}else {
				//takeLeftEndCircle
				System.out.println("R-L");

				float x1 = centerRStart.getX();
				float y1 = centerRStart.getZ();
				float x2 = centerLEnd.getX();
				float y2 = centerLEnd.getZ();
				float r = turningRadius;
				float d = centerLEnd.subtract(centerRStart).euclideanLength()/2;


				float theta1 =  (-4*d*d + 16*r*r - 3*x1*x1 + 2*x1*x2 + x2*x2 -3*y1*y1 +2*y1*y2 + y2*y2)/(4*(x1-x2));
				float theta = 4*d*d*(y1 - y2) + 16*r*r*(y2-y1) +  x1*x1* (3*y1  +  y2) + x2*x2*(3*y1+y2) +y1*y2*y2 - 5*y1*y1*y2 + 3*y1*y1*y1 + y2*y2*y2 - 6*x1*x2*y1 - 2*x1*x2*y2;
				float theta5 = (float) Math.sqrt(((4*d*d +16 *d*r + 16*r*r - x1*x1 + 2*x1*x2 - x2*x2-y1*y1+2*y1*y2-y2*y2)*(-4*d*d + 16*d*r-16*r*r + x1*x1 -2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2))/4);
				float theta6 = 2*(x2-x1)*theta5;
				float theta2 = theta + theta6;
				float theta3 = theta - theta6;
				float theta13 = x1*x1 - 2*x1*x2+x2*x2+y1*y1-2*y1*y2+y2*y2;
				float theta4 = 4*(x1-x2)*theta13;

				float tx1 = -theta1 - (y1-y2)*theta2/theta4;
				float ty1 = theta2/(4*theta13);
				float tx2 = -theta1 - (y1-y2)*theta3/theta4;
				float ty2 = theta3/(4*theta13);

				Vector t1 = new Vector(tx1,0,ty1);
				Vector offset1 = centerLEnd.subtract(t1);
				t1 = centerRStart.add(t1.subtract(centerRStart).constantProduct(.5f));
				Vector t2 = new Vector(tx2,0,ty2);
				Vector offset2 = centerLEnd.subtract(t2);
				t2 = centerRStart.add(t2.subtract(centerRStart).constantProduct(.5f));

				centerRStart.printVector("start:  ");
				centerLEnd.printVector("end:  ");
//				t1.printVector("t1:  ");
//				t2.printVector("t2:  ");



				Vector r1 = t1.add(offset1);
				Vector r2 = t2.add(offset2);
//				r1.printVector("r1:  ");
//				r2.printVector("r2:  ");



				//get path on first circle

				Vector a1 = t2.subtract(centerRStart);
				Vector a2 = startingPoint.subtract(centerRStart);
				float angle = a2.angleBetween(a1);
				float arclenght = (float) (turningRadius*angle/(2*Math.PI));
				float nbPathentry = arclenght/20;
				float stepAngle = angle/nbPathentry;

				Vector nzAxis = new Vector(0,0,-1);
				Vector Airportaxis = new Vector(from.getCenterToRunway0X(),0,from.getCenterToRunway0Z());
				float bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() < 0){
					bias -= Math.PI;
				}
				System.out.println(bias);
				float signx = -1;
				float signz = -1;

				for (int i=0;i<nbPathentry;i++) {
					float x = (float) (centerRStart.getX() + signx * Math.cos(bias + stepAngle * i)*turningRadius);
					float y = (float) (centerRStart.getZ() + signz * Math.sin(bias + stepAngle * i)*turningRadius);
					Vector pathEntry = new Vector(x,20,y);
					path.add(pathEntry);

				}
				float x = (float) (centerRStart.getX() + signx * Math.cos(bias + angle)*turningRadius);
				float y = (float) (centerRStart.getZ() + signz * Math.sin(bias + angle)*turningRadius);
				Vector pathEntry = new Vector(x,20,y);
				path.add(pathEntry);

				// get path on tangent

				Vector tangent = r2.subtract(t2);
				nbPathentry = tangent.euclideanLength()/20;
				for (int i= 1; i <nbPathentry;i++) {
					float partOfTangent = i/nbPathentry;
					pathEntry = t2.add(tangent.constantProduct(partOfTangent));
					pathEntry = new Vector(pathEntry.getX(),20,pathEntry.getZ());
					path.add(pathEntry);
				}


				// get path on second circle

				a1 = r2.subtract(centerLEnd);
				a2 = endPoint.subtract(centerLEnd);
				angle = a2.angleBetween(a1);

				arclenght = (float) (turningRadius*angle/(2*Math.PI));
				nbPathentry = arclenght/20;
				stepAngle = angle/nbPathentry;
				nzAxis = new Vector(0,0,-1);
				Airportaxis = new Vector(to.getCenterToRunway0X(),0,to.getCenterToRunway0Z());
				bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() < 0) {
					bias -= Math.PI;
				}
				System.out.println(bias);
				signx = 1;
				signz = 1;
				for (int i= 1; i <nbPathentry;i++) {
					x = (float) (centerLEnd.getX() + signx * Math.cos(bias + angle - stepAngle * i)*turningRadius);
					y = (float) (centerLEnd.getZ() + signz * Math.sin(bias + angle - stepAngle * i)*turningRadius);
					pathEntry = new Vector(x,20,y);
					path.add(pathEntry);
				}

				path.add(endPoint);

			}
		}else {
			//takeLeftCircle
			if(centerLStart.calculateDistance(centerREnd) < centerLStart.calculateDistance(centerLEnd)) {
				//takeRightEndCircle
				System.out.println("L-R");




				float x1 = centerLStart.getX();
				float y1 = centerLStart.getZ();
				float x2 = centerREnd.getX();
				float y2 = centerREnd.getZ();
				float r = turningRadius;
				float d = centerLEnd.subtract(centerLStart).euclideanLength()/2;


				float theta1 =  (-4*d*d + 16*r*r - 3*x1*x1 + 2*x1*x2 + x2*x2 -3*y1*y1 +2*y1*y2 + y2*y2)/(4*(x1-x2));
				float theta = 4*d*d*(y1 - y2) + 16*r*r*(y2-y1) +  x1*x1* (3*y1  +  y2) + x2*x2*(3*y1+y2) +y1*y2*y2 - 5*y1*y1*y2 + 3*y1*y1*y1 + y2*y2*y2 - 6*x1*x2*y1 - 2*x1*x2*y2;
				float theta5 = (float) Math.sqrt(((4*d*d +16 *d*r + 16*r*r - x1*x1 + 2*x1*x2 - x2*x2-y1*y1+2*y1*y2-y2*y2)*(-4*d*d + 16*d*r-16*r*r + x1*x1 -2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2))/4);
				float theta6 = 2*(x2-x1)*theta5;
				float theta2 = theta + theta6;
				float theta3 = theta - theta6;
				float theta13 = x1*x1 - 2*x1*x2+x2*x2+y1*y1-2*y1*y2+y2*y2;
				float theta4 = 4*(x1-x2)*theta13;

				float tx1 = -theta1 - (y1-y2)*theta2/theta4;
				float ty1 = theta2/(4*theta13);
				float tx2 = -theta1 - (y1-y2)*theta3/theta4;
				float ty2 = theta3/(4*theta13);

				Vector t1 = new Vector(tx1,0,ty1);
				Vector offset1 = centerREnd.subtract(t1);
				t1 = centerLStart.add(t1.subtract(centerLStart).constantProduct(.5f));
				Vector t2 = new Vector(tx2,0,ty2);
				Vector offset2 = centerREnd.subtract(t2);
				t2 = centerLStart.add(t2.subtract(centerLStart).constantProduct(.5f));

				centerLStart.printVector("start:  ");
				centerREnd.printVector("end:  ");
				t1.printVector("t1:  ");
				t2.printVector("t2:  ");



				Vector r1 = t1.add(offset1);
				Vector r2 = t2.add(offset2);
				r1.printVector("r1:  ");
				r2.printVector("r2:  ");


				//testing

				r2 = r1;
				t2 = t1;
				//testing




				//get path on first circle

				Vector a1 = t2.subtract(centerLStart);
				Vector a2 = startingPoint.subtract(centerLStart);
				float angle = a2.angleBetween(a1);
				float arclenght = (float) (turningRadius*angle/(2*Math.PI));
				float nbPathentry = arclenght/20;
				float stepAngle = angle/nbPathentry;

				Vector nzAxis = new Vector(0,0,-1);
				Vector Airportaxis = new Vector(from.getCenterToRunway0X(),0,from.getCenterToRunway0Z());
				float bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() > 0){
					bias -= Math.PI;
				}
				System.out.println(bias);
				float signx = 1;
				float signz = -1;

				for (int i=0;i<nbPathentry;i++) {
					float x = (float) (centerLStart.getX() + signx * Math.cos(bias + stepAngle * i)*turningRadius);
					float y = (float) (centerLStart.getZ() + signz * Math.sin(bias + stepAngle * i)*turningRadius);
					Vector pathEntry = new Vector(x,20,y);
					path.add(pathEntry);
				}
				float x = (float) (centerLStart.getX() + signx * Math.cos(bias + angle)*turningRadius);
				float y = (float) (centerLStart.getZ() + signz * Math.sin(bias + angle)*turningRadius);
				Vector pathEntry = new Vector(x,20,y);
				path.add(pathEntry);

				// get path on tangent

				Vector tangent = r2.subtract(t2);
				nbPathentry = tangent.euclideanLength()/20;
				for (int i= 1; i <nbPathentry;i++) {
					float partOfTangent = i/nbPathentry;
					pathEntry = t2.add(tangent.constantProduct(partOfTangent));
					pathEntry = new Vector(pathEntry.getX(),20,pathEntry.getZ());
					path.add(pathEntry);
				}


				// get path on second circle

				a1 = r2.subtract(centerREnd);
				a2 = endPoint.subtract(centerREnd);
				angle = a2.angleBetween(a1);

				arclenght = (float) (turningRadius*angle/(2*Math.PI));
				nbPathentry = arclenght/20;
				stepAngle = angle/nbPathentry;
				nzAxis = new Vector(0,0,-1);
				Airportaxis = new Vector(to.getCenterToRunway0X(),0,to.getCenterToRunway0Z());
				bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() > 0) {
					bias -= Math.PI;
				}
				System.out.println(bias);
				signx = -1;
				signz = 1;
				for (int i= 1; i <nbPathentry;i++) {
					x = (float) (centerREnd.getX() + signx * Math.cos(bias + angle - stepAngle * i)*turningRadius);
					y = (float) (centerREnd.getZ() + signz * Math.sin(bias + angle - stepAngle * i)*turningRadius);
					pathEntry = new Vector(x,20,y);
					path.add(pathEntry);
				}

				path.add(endPoint);

			}else {
				//takeLeftEndCircle

				System.out.println("L-L");


				float x1 = centerLStart.getX();
				float y1 = centerLStart.getZ();
				float x2 = centerLEnd.getX();
				float y2 = centerLEnd.getZ();
				float r = turningRadius;
				Vector offset = centerLEnd.subtract(centerLStart);
				Vector toT = offset.constantProduct(r/offset.euclideanLength());
				toT = new Vector(-toT.getZ(),0,toT.getX());
				Vector t1 = centerLStart.add(toT);
				Vector t2 = centerLStart.subtract(toT);



//				centerLStart.printVector("start:  ");
//				centerLEnd.printVector("end:  ");
//				t1.printVector("t1:  ");
//				t2.printVector("t2:  ");



				Vector r1 = t1.add(offset);
				Vector r2 = t2.add(offset);
//				r1.printVector("r1:  ");
//				r2.printVector("r2:  ");

                t2 = t1;
				r2 = r1;

				//get path on first circle
//todo add support for turned airports
				Vector a1 = t2.subtract(centerLStart);
				Vector a2 = startingPoint.subtract(centerLStart);
				float angle = a2.angleBetween(a1);
				float arclenght = (float) (turningRadius*angle/(2*Math.PI));
				float nbPathentry = arclenght/20;
				float stepAngle = angle/nbPathentry;

                Vector nzAxis = new Vector(0,0,-1);
				Vector Airportaxis = new Vector(from.getCenterToRunway0X(),0,from.getCenterToRunway0Z());
				float bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() > 0){
					bias -= Math.PI;
				}
				System.out.println(bias);
                float signx = 1;
                float signz = -1;

				for (int i=0;i<nbPathentry;i++) {
					float x = (float) (centerLStart.getX() + signx * Math.cos(bias + stepAngle * i)*turningRadius);
					float y = (float) (centerLStart.getZ() + signz * Math.sin(bias + stepAngle * i)*turningRadius);
					Vector pathEntry = new Vector(x,20,y);
					path.add(pathEntry);

				}
				float x = (float) (centerLStart.getX() + signx * Math.cos(bias + angle)*turningRadius);
				float y = (float) (centerLStart.getZ() + signz * Math.sin(bias + angle)*turningRadius);
				Vector pathEntry = new Vector(x,20,y);
				path.add(pathEntry);

				// get path on tangent

				Vector tangent = r2.subtract(t2);
				nbPathentry = tangent.euclideanLength()/20;
				for (int i= 1; i <nbPathentry;i++) {
					float partOfTangent = i/nbPathentry;
					pathEntry = t2.add(tangent.constantProduct(partOfTangent));
					pathEntry = new Vector(pathEntry.getX(),20,pathEntry.getZ());
					path.add(pathEntry);
				}


				// get path on second circle

				a1 = r2.subtract(centerLEnd);
				a2 = endPoint.subtract(centerLEnd);
				angle = a2.angleBetween(a1);

				arclenght = (float) (turningRadius*angle/(2*Math.PI));
				nbPathentry = arclenght/20;
				stepAngle = angle/nbPathentry;
				nzAxis = new Vector(0,0,-1);
				Airportaxis = new Vector(to.getCenterToRunway0X(),0,to.getCenterToRunway0Z());
				bias = nzAxis.angleBetween(Airportaxis);
				if (from.getCenterToRunway0X() > 0) {
					bias -= Math.PI;
				}
				System.out.println(bias);
				signx = 1;
				signz = 1;
				for (int i= 1; i <nbPathentry;i++) {
					x = (float) (centerLEnd.getX() + signx * Math.cos(bias + angle - stepAngle * i)*turningRadius);
					y = (float) (centerLEnd.getZ() + signz * Math.sin(bias + angle - stepAngle * i)*turningRadius);
					pathEntry = new Vector(x,20,y);
					path.add(pathEntry);
				}

				path.add(endPoint);



			}
		}

		


        System.out.println("path: "+path);
		return new PathImplementation(path);
	}
	
	private Airport from;
	private Airport to;
	private AutopilotImplementation drone = null;
	private int gateFrom;
	private int gateTo;
	private PathImplementation path;
}


