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
		updateMap(char curPos, char curHead, int isWall);
		g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		// Insert your code here...
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
	
	public static void updateMap(char curPos, char curHead, int isWall){ 
		/***
		 * Inputs: current Position, current Heading, an integer isWall which is 1 if sensor detects a wall, 0 otherwise
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map 
		***/
		
		
		int xpos = 0;
		int ypos = 0;
		
		switch(curPos){
			case'A': case'B': case'C': case'D': case'E':
				ypos = 1;
				break;
			case'F': case'G': case'H': case'I': case'J':
				ypos = 3;
				break;
			case'K': case'L': case'M': case'N': case'O':
				ypos = 5;
				break;
			case'P': case'Q': case'R': case'S': case'T':
				ypos = 7;
				break;
			case'U': case'V': case'W': case'X': case'Y':
				ypos = 10;
				break;
				}
		
		switch(curPos){
			case'A': case'F': case'K': case'P': case'U':
				xpos = 1;
				break;
			case'B': case'G': case'L': case'Q': case'V':
				xpos = 3;
				break;
			case'C': case'H': case'M': case'R': case'W':
				xpos = 5;
				break;
			case'D': case'I': case'N': case'S': case'X':
				xpos = 7;
				break;
			case'E': case'J': case'O': case'T': case'Y':
				xpos = 10;
				break;
				}
				
		int[] walldist = new int[3];
		// walldist[0] is right, [1] is front, [2] is left
		walldist = getwalldist();
		switch(curHead){
			// Question: What do we do when invadid measurement? ignore, measure again or mark direction?
			case 'L':	//The robot is facing negative x
				// Check if measurements are feasable (must be in boundaries)
				if (ypos-walldist[0]>=1 && walldist[0] != 0){
					my_map[xpos][ypos-walldist[1]] = (char)isWall;
				} 
				if (xpos-walldist[1]>=1 && walldist[1] != 0){
					my_map[xpos-walldist[2]][ypos] = (char)isWall;
				}
				if(ypos+walldist[2] <= 9 && walldist[2] != 0){
					my_map[xpos][ypos+walldist[3]] = (char)isWall;
				}
				break;
			case 'R':
				if (ypos+walldist[0]<=9 && walldist[0] != 0){
					my_map[xpos][ypos+walldist[1]] = (char)isWall;
				} 
				if (xpos+walldist[1]<=9 && walldist[1] != 0){
					my_map[xpos+walldist[2]][ypos] = (char)isWall;
				}
				if(ypos-walldist[2] >= 1 && walldist[2] != 0){
					my_map[xpos][ypos-walldist[3]] = (char)isWall;
				}
				break;
			case 'U':
					if (xpos+walldist[0]<=9 && walldist[0] != 0){
						my_map[xpos+walldist[1]][ypos] = (char)isWall;
					} 
					if (ypos-walldist[1]>=1 && walldist[1] != 0){
						my_map[xpos][ypos-walldist[1]] = (char)isWall;
					}
					if(xpos-walldist[2] >= 1 && walldist[2] != 0){
						my_map[xpos-walldist[2]][ypos] = (char)isWall;
					}
				break;
			case 'D':
					if (xpos-walldist[0]>=1 && walldist[0] != 0){
						my_map[xpos-walldist[1]][ypos] = (char)isWall;
					} 
					if (ypos+walldist[1]<=9 && walldist[1] != 0){
						my_map[xpos][ypos+walldist[1]] = (char)isWall;
					}
					if(xpos+walldist[2] <= 9 && walldist[2] != 0){
						my_map[xpos+walldist[2]][ypos] = (char)isWall;
					}
				break;
				}		
	}
	
	public static int[] getwalldist(){
		//EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		//while
		int[] walldist = new int[3];
		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S4);
		int sampleSize = sonic.sampleSize();
		float[] sonicsample = new float[sampleSize];
		
		Motor.A.setSpeed(60);
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
			Motor.A.rotate(-92);
		}
		Motor.A.rotate(-195);
		sonic.close();
		return walldist;
	}

	// moves robot one step to the front
	public static void move(){ 

		// get Linedetector measurement
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3); LCD.clear();
		int sampleSize_color = color.sampleSize();
		float[] line_measurement = new float[sampleSize_color]; 
		color.getColorIDMode().fetchSample(line_measurement, 0); 
		//LCD.clear();
		System.out.println(line_measurement[0]);

		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S4);
		int sampleSize_sonic = sonic.sampleSize();
		float[] sonicsample = new float[sampleSize_sonic];
		sonic.fetchSample(sonicsample,0);
		float distance_cm = sonicsample[0]*100;

		float A_init = Motor.A.getTachoCount();
		float B_init = Motor.B.getTachoCount();
		float frontwalldist = 20;
		float eps = 2;
		float intervall_dist = 30;
		while (distance_cm > frontwalldist+eps || ){
			sonic.fetchSample(sonicsample,0);
			distance_cm = sonicsample[0]*100;
		}

		sonic.close();
		color.close();

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
	
	public static char[] getNeighbours(char position, char[][] map, Map<Character, int[]> char_to_position){ 
		/***
		 * Inputs: position (char identifier of position in map we want to get the neighbours of)
		 * 		   map (my_map variable above)
		 * 		   char_to_position (hash map, see explanation in initializaMap() )
		 * Outputs: character array size between 1 and 4 of the neighbours (e.g. if we query H, return char will be 'C','I','M','G')
		 * Function: Return neighbors to queried node 
		***/
		
		// Insert your code here...
		char[] neighbours = new char[4]; // UPDATE THIS
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
