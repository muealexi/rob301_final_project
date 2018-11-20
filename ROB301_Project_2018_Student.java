import java.util.ArrayList;
//import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.concurrent.TimeUnit;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class ROB301_Project_2018_Student {
	static int asci_count = 0; // ASCII counter
	static int[] coord = new int [2]; // Keep track of coordinates
	static Map<Character, int[]> char_to_position; // Hash map maps node with given name to coordinate on map
	static char[][] my_map; // Stores maze map
	
	public static void main(String[] args) throws Exception{  
		int sizeMapX = 11; int sizeMapY = 11;
		char curPos = 'A'; // Start position of robot (to be updated)
		char curHead = 'R'; // Start orientation of robot (either 'U', 'D', 'L', 'R') (to be updated)
		char goalPos = 'Y'; // Final position the robot needs to reach
		char goalHead = 'U'; // Final orientation the robot needs to reach (either 'U', 'D', 'L', 'R')
		
		List<Character> optPath; // Optimal path
		
		initializeMap(); // Initialize map with no walls
		Graph g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		
		my_map[1][6] = '1'; // Add a wall to the map (for demo)
		updateMap(curPos,curHead);
		g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		while (curPos != goalPos && curHead != goalHead){
			updateMap(curPos, curHead);
			optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal

			// needs the next Heading as input
			curHead = rotateRobot(curHead, nextHead);
			int nextwall = getnextwall(curPos, curHead);
			move(nextwall);
			curPos = moveRobot(my_map, curPos, curHead);
		}

	}
	
	public static void initializeMap(){ 
		/* Map should look like:
		 *  ZZZZZZZZZZZ
			ZA0B0C0D0EZ
			Z0Z0Z0Z0Z0Z
			ZF0G0H0I0JZ
			Z0Z0Z0Z0Z0Z
			ZK0L0M0N0OZ
			Z0Z0Z0Z0Z0Z
			ZP0Q0R0S0TZ
			Z0Z0Z0Z0Z0Z
			ZU0V0W0X0YZ
			ZZZZZZZZZZZ
			
			Hash map char_to_postion is like a dictionary relating characters (e.g. 'A') to coordinates (e.g. [1,1]) in my_map
			
			Note that positive X is right and positive Y is down
			Z character is a null entry of the map
			Alphabetical characters from A to Y are potential positions the robot can be in
			Numerical characters can hold either 0 or 1 (to signify empty space or wall respectively between its neighbouring positions)
		 */
		
		char_to_position = new HashMap<Character, int[]>(); // Create hash from character to position in map
		my_map = new char[11][11]; // Create map from position to character (i.e. regular map)
		char letter; // Holds character corresponding to a position in the map
		// Populate entire array with Z
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				my_map[i][j] = 'Z';
			}
		}
		// Populate inner map area with 0's to signify free path between robot positions
		for(int i = 1; i < 10; i++){
			for(int j =1; j < 10; j ++){
				my_map[i][j] = '0';
			}
		}
		// Populate cells from A-Y where robot will go
		for(int i = 1; i < 10; i+=2){
			for(int j =1; j < 10; j +=2){
				int[] coord = new int [2]; // Must create new array object so since hash map points all keys to same 
				letter = (char)(65+asci_count);
				my_map[i][j] = letter;
				coord [0] = i; coord[1] = j;
				char_to_position.put(letter, coord);
				asci_count++;
			}
		}
		
		//Rest of map is padded with Z character to make parsing the map easier to implement
		for(int i = 2; i < 10; i+=2){
			for(int j =2; j < 10; j +=2){
				my_map[i][j] = 'Z';
			}
		}
	}

	public static void updateMap(char curPos, char curHead){ 
		/***
		 * Inputs: current Position, current Heading, an integer isWall which is 1 if sensor detects a wall, 0 otherwise
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map 
		***/
		
		int[] cur_coord = char_to_position.get(curPos);

		int xpos = cur_coord[0];
		int ypos = cur_coord[1];
				
		int[] walldist = new int[3];
		// walldist[0] is right, [1] is front, [2] is left
		walldist = getwalldist();
		switch(curHead){
			// Question: What do we do when invadid measurement? ignore, measure again or mark direction?
			case 'L':	//The robot is facing negative x
				// Check if measurements are feasable (must be in boundaries of map)
				if (ypos-walldist[0]>=1 && walldist[0] != 0){
					my_map[xpos][ypos-walldist[1]] = (char)1;
				} 
				if (xpos-walldist[1]>=1 && walldist[1] != 0){
					my_map[xpos-walldist[2]][ypos] = (char)1;
				}
				if(ypos+walldist[2] <= 9 && walldist[2] != 0){
					my_map[xpos][ypos+walldist[3]] = (char)1;
				}
				break;
			case 'R':
				if (ypos+walldist[0]<=9 && walldist[0] != 0){
					my_map[xpos][ypos+walldist[1]] = (char)1;
				} 
				if (xpos+walldist[1]<=9 && walldist[1] != 0){
					my_map[xpos+walldist[2]][ypos] = (char)1;
				}
				if(ypos-walldist[2] >= 1 && walldist[2] != 0){
					my_map[xpos][ypos-walldist[3]] = (char)1;
				}
				break;
			case 'U':
					if (xpos+walldist[0]<=9 && walldist[0] != 0){
						my_map[xpos+walldist[1]][ypos] = (char)1;
					} 
					if (ypos-walldist[1]>=1 && walldist[1] != 0){
						my_map[xpos][ypos-walldist[1]] = (char)1;
					}
					if(xpos-walldist[2] >= 1 && walldist[2] != 0){
						my_map[xpos-walldist[2]][ypos] = (char)1;
					}
				break;
			case 'D':
					if (xpos-walldist[0]>=1 && walldist[0] != 0){
						my_map[xpos-walldist[1]][ypos] = (char)1;
					} 
					if (ypos+walldist[1]<=9 && walldist[1] != 0){
						my_map[xpos][ypos+walldist[1]] = (char)1;
					}
					if(xpos+walldist[2] <= 9 && walldist[2] != 0){
						my_map[xpos+walldist[2]][ypos] = (char)1;
					}
				break;
				}		
	}
	
		// moves robot one step to the front
		public static void move(int nextwall){ 

			// Color Setup
			EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3); 
			int sampleSize_color = color.sampleSize();
			float[] line_measurement = new float[sampleSize_color]; 
			color.getColorIDMode().fetchSample(line_measurement, 0); 
			//LCD.clear();
			//System.out.println(line_measurement[0]);
	
			// Sonic Setup
			EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S4);
			int sampleSize_sonic = sonic.sampleSize();
			float[] sonicsample = new float[sampleSize_sonic];
			sonic.fetchSample(sonicsample,0);
			float distance_cm = sonicsample[0]*100;
	
			// sonic constants
			float frontwalldist = 20 + 30*nextwall; // true distance when color sensor is exactly on cross junction
			float eps = 2; // threshold to stop PID before colorsensor reaches crossjunction
	
			// Motor setup
			int basespeed = 60;
			Motor.B.setSpeed(basespeed); // B is left motor
			Motor.C.setSpeed(basespeed); // C is right motor
			//float A_init = Motor.B.getTachoCount();
			//float B_init = Motor.C.getTachoCount();
	
			// PID Constants
			float eq_point = 0.25;
			float err;
			float err_prev = 0;
			float err_dot;
			float u;
			int kp = 100;
			int kd = 50;
	
			// if condition not fullfilled from beginning, then maybe front wall measurement again?
			while (distance_cm > frontwalldist){
				
				if(distance_cm > frontwalldist+eps){
					color.getColorIDMode().fetchSample(line_measurement, 0); // 0 is on line, 0.6 is outside
					err = line_measurement[0] - eq_point; // err < 0 -> too far on line, err > 0 --> too far outside line 
					err_dot = err - err_prev;
					u = kp * err + kd * err_dot;
		
					// Sensor is on the right side of the Line
					Motor.B.forward(basespeed-(int)u);
					Motor.C.forward(basespeed+(int)u);
	
					err_prev = err;
	
					// only measure distance if we are near the line and not too twisted to avoid bad measurements
					if (err < 0.1 && err_dot < 0.1){
						sonic.fetchSample(sonicsample,0);
						distance_cm = sonicsample[0]*100;
					}
				} else{
					Motor.B.forward(basespeed);
					Motor.C.forward(basespeed);
					sonic.fetchSample(sonicsample,0);
					distance_cm = sonicsample[0]*100;
				}
			}
			sonic.close();
			color.close();
		}

	// Updates Position after move() command -> Moves current position one in direction of current heading
	public static char moveRobot(char[][] map, char curPos, char curHead){
		
		int[] cur_coord = char_to_position.get(curPos);
		int xpos = cur_coord[0];
		int ypos = cur_coord[1];
		char next_curPos;
				
		switch(curHead){
			case 'L':	//The robot is facing negative x
				next_curPos = map[xpos-2][ypos];
				break;
			case 'R':
				next_curPos = map[xpos+2][ypos];
				break;
			case 'U':
				next_curPos = map[xpos][ypos-2];
				break;
			case 'D':
				next_curPos = map[xpos][ypos+2];
				break;
				}	
				
		return next_curPos;
	}

	public static char rotateRobot(char curHead, char nextHead){

		// Motor setup
		int basespeed = 60;
		Motor.B.setSpeed(basespeed); // B is left motor
		Motor.C.setSpeed(basespeed); // C is right motor
		float fullrot_aroundwheel = 4.2727 * 360;
		float quarterrot = fullrot_aroundwheel*0.25;
		float fullrot_onspot = 2.13636 * 360;
		float halfrot_onspot = fullrot_onspot*0.5;

		int curhead2ind;
		int nexthead2ind;

		switch(curHead){
			case "U": curhead2ind = 0; break;
			case "R": curhead2ind = 1; break;
			case "D": curhead2ind = 2; break;
			case "L": curhead2ind = 3; break;
		}
		switch(nextHead){
			case "U": nexthead2ind = 0; break;
			case "R": nexthead2ind = 1; break;
			case "D": nexthead2ind = 2; break;
			case "L": nexthead2ind = 3; break;
		}
		int diff = nexthead2ind-curhead2ind;

		if (diff == 1 || diff == -3){
			// turn Robot 90deg clockwise, not moving right wheel
			Motor.B.rotate((int)quarterrot);
		} else if(diff == 2 || diff == -2){
			// turn Robot 180deg clockwise on spot			
			Motor.B.rotate((int)halfrot_onspot);
			Motor.C.rotate(-(int)halfrot_onspot);
		} else if(diff == 3 || diff == -1){
			// turn Robot 90deg counter-clockwise, not moving left wheel
			Motor.C.rotate((int)quarterrot);
		}

		return nextHead;
	}

	public static int[] getwalldist(){
		//EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		//while
		int[] walldist = new int[3];
		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S4);
		int sampleSize = sonic.sampleSize();
		float[] sonicsample = new float[sampleSize];
		
		Motor.A.setSpeed(40);
		int box = 30;
		int sidedist = 15+5;
		
		for(int i=0; i<3; i+=1){
			sonic.fetchSample(sonicsample,0);
			sonicsample[0] = sonicsample[0]*100;
			
			// because of backward displacement of sensor offset second (forward) measurement:
			if (i == 1){
				sonicsample[0] -= 5;
			}
			
			// measure first right, then forward then left distance to wall and safe
			if(sonicsample[0] > 5 && sonicsample[0] < sidedist){
				walldist[i] = 1;
			} else if (sonicsample[0] > sidedist && sonicsample[0] < sidedist+box){
				walldist[i] = 3;
			} else if (sonicsample[0] > sidedist+box && sonicsample[0] < sidedist+2*box){
				walldist[i] = 5;
			} else if (sonicsample[0] > sidedist+2*box && sonicsample[0] < sidedist+3*box){
				walldist[i] = 7;
			} else {
				walldist[i] = 0;	// invalid measurement
			}

			// measures first forward, then right, then left
			if (i==0){
				Motor.A.rotate(92);
			} else if (i==1){
				Motor.A.rotate(-195);
			} else {
				Motor.A.rotate(92);
			}
		}

		// reorder walldist array
		int placeholder = walldist[0];
		walldist[0] = walldist[1];
		walldist[1] = walldist[0];
		sonic.close();
		return walldist; // 0: right measurement, 1: front, 2: Left
	}

	// calculates where next wall is (0: wall is just in front, 1: 1 square between robot and wall, 2:...)
	public static void getnextwall(char curPos, char curHead){
		int nextwall = 0;
		int i = 1;
		while (nextwall == 0){
			if (curHead == 'L' && my_map[xpos-(i*2-1)][ypos] != '0'){
				nextwall = i;
			} else if (curHead == 'R' && my_map[xpos+(i*2-1)][ypos] != '0'){
				nextwall = i;
			} else if (curHead == 'U' && my_map[xpos][ypos-(i*2-1)] != '0'){
				nextwall = i;
			} else if (curHead == 'D' && my_map[xpos][ypos+(i*2-1)] != '0'){
				nextwall = i;
			}
			if (i > 9){
				System.out.print("No next wall detected");
				break;
			}
			i = i+1;
		}
		nextwall -= 1; // easier for calculation in other function
	}
	public static Graph getGraph(char[][] map, int sizeX, int sizeY, Map<Character, int[]> char_to_position){ 
		// Iterate through each robot position of the map
		char[] neighbours;
		Graph g = new Graph();
		char letter;
		for(int i = 1; i < sizeX-1; i+=2){
			for(int j =1; j < sizeY-1; j +=2){
				letter = map[i][j]; // Get current letter we're on and create edges from this on the graph
				neighbours = getNeighbours(letter, map, char_to_position);
				ArrayList<Vertex> vertices = new ArrayList<Vertex>();
				for(int k=0; k < 4; k++){ // Iterate over all neighbours of current position in map
					if(neighbours[k] != 'Z'){
						vertices.add(new Vertex(neighbours[k],1));
					}else{
						break;
					}
				}
				g.addVertex(letter, vertices); // Add list of neighbouring vertices to graph
			}
		}
		return g;
	}
	
	// gets 4 neighbours from in this order: U,R,D,L
	public static char[] getNeighbours(char position, char[][] map, Map<Character, int[]> char_to_position){ 
		/***
		 * Inputs: position (char identifier of position in map we want to get the neighbours of)
		 * 		   map (my_map variable above)
		 * 		   char_to_position (hash map, see explanation in initializaMap() )
		 * Outputs: character array size between 1 and 4 of the neighbours (e.g. if we query H, return char will be 'C','I','M','G')
		 * Function: Return neighbors to queried node 
		***/
		int[] cur_coord = char_to_position.get(position);
		char[] neighbours = new char[4]; // UPDATE THIS
		neighbours[0] = map[cur_coord[0]][cur_coord[1]-2];
		neighbours[1] = map[cur_coord[0]+2][cur_coord[1]];
		neighbours[2] = map[cur_coord[0]][cur_coord[1]+2];
		neighbours[3] = map[cur_coord[0]-2][cur_coord[1]];

		return neighbours;
	}
	
	public static void printMap(char[][] map){
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				System.out.print(map[i][j]);
			}
			System.out.println("");
		}
	}
}

// DO NOT CHANGE FOLLOWING CODE. Path planning implementation
class Vertex implements Comparable<Vertex> {
	
	private Character id;
	private Integer distance;
	
	public Vertex(Character id, Integer distance) {
		super();
		this.id = id;
		this.distance = distance;
	}

	public Character getId() {
		return id;
	}

	public Integer getDistance() {
		return distance;
	}

	public void setId(Character id) {
		this.id = id;
	}

	public void setDistance(Integer distance) {
		this.distance = distance;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((distance == null) ? 0 : distance.hashCode());
		result = prime * result + ((id == null) ? 0 : id.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vertex other = (Vertex) obj;
		if (distance == null) {
			if (other.distance != null)
				return false;
		} else if (!distance.equals(other.distance))
			return false;
		if (id == null) {
			if (other.id != null)
				return false;
		} else if (!id.equals(other.id))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "Vertex [id=" + id + ", distance=" + distance + "]";
	}

	@Override
	public int compareTo(Vertex o) {
		if (this.distance < o.distance)
			return -1;
		else if (this.distance > o.distance)
			return 1;
		else
			return this.getId().compareTo(o.getId());
	}
	
}

class Graph { 
	public final Map<Character, List<Vertex>> vertices;
	
	public Graph() {
		this.vertices = new HashMap<Character, List<Vertex>>();
	}
	
	public void addVertex(Character character, List<Vertex> vertex) {
		this.vertices.put(character, vertex);
	}
	
	public void createHashMap(){
		
	}
	
	public List<Character> getShortestPath(Character start, Character finish) {
		final Map<Character, Integer> distances = new HashMap<Character, Integer>();
		final Map<Character, Vertex> previous = new HashMap<Character, Vertex>();
		PriorityQueue<Vertex> nodes = new PriorityQueue<Vertex>();
		
		for(Character vertex : vertices.keySet()) {
			if (vertex == start) {
				distances.put(vertex, 0);
				nodes.add(new Vertex(vertex, 0));
			} else {
				distances.put(vertex, Integer.MAX_VALUE);
				nodes.add(new Vertex(vertex, Integer.MAX_VALUE));
			}
			previous.put(vertex, null);
		}
		
		while (!nodes.isEmpty()) {
			Vertex smallest = nodes.poll();
			if (smallest.getId() == finish) {
				final List<Character> path = new ArrayList<Character>();
				while (previous.get(smallest.getId()) != null) {
					path.add(smallest.getId());
					smallest = previous.get(smallest.getId());
				}
				return path;
			}

			if (distances.get(smallest.getId()) == Integer.MAX_VALUE) {
				break;
			}
						
			for (Vertex neighbor : vertices.get(smallest.getId())) {
				Integer alt = distances.get(smallest.getId()) + neighbor.getDistance();
				if (alt < distances.get(neighbor.getId())) {
					distances.put(neighbor.getId(), alt);
					previous.put(neighbor.getId(), smallest);
					
					forloop:
					for(Vertex n : nodes) {
						if (n.getId() == neighbor.getId()) {
							nodes.remove(n);
							n.setDistance(alt);
							nodes.add(n);
							break forloop;
						}
					}
				}
			}
		}
		
		return new ArrayList<Character>(distances.keySet());
	}
}
