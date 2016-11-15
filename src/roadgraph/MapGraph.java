/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 * Edges are one way roads from Start to End Nodes 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 * Edges are one way roads from Start to End Nodes
 *
 */
public class MapGraph {
	// private data of MapGraph
	private HashSet <MapEdge> edges;   // store the edges
	private HashMap <GeographicPoint, MapNode> locationNodeMap; // we need a mapping from locations to Nodes
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		locationNodeMap = new HashMap <GeographicPoint, MapNode>();
		edges = new HashSet <MapEdge>();  // create new empty storage structures
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return locationNodeMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return locationNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

		
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null) {
			return false;
		}
		else {
			MapNode node = locationNodeMap.get(location);
			if (node == null) {
				node = new MapNode(location);
				locationNodeMap.put(location, node);
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if (length < 0) {
			throw new IllegalArgumentException("Length must be nonnegative!");
		}
		
		if (from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException("check for null argument(s)!");
		}
		
		if (!locationNodeMap.containsKey(from) || !locationNodeMap.containsKey(to)) {
			throw new IllegalArgumentException("check your start and end nodes!");
		}
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(edge);
		locationNodeMap.get(from).addEdge(edge);
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Error: null start or goal:  No path exists.");
			return new LinkedList<GeographicPoint>();
			}
		
		MapNode startNode = locationNodeMap.get(start);
		MapNode goalNode = locationNodeMap.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Error: start or goal not in graph: No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}

		// construct the path
		return constructPath(startNode, goalNode, parentMap);
						
	}
	
	private List<GeographicPoint> constructPath(MapNode start, MapNode goal,
			HashMap<MapNode, MapNode> parentMap)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goal;
		GeographicPoint currPoint = goal.getLocation();
		while (currPoint != start.getLocation()) {
			path.addFirst(currPoint);
			curr = parentMap.get(curr);
			currPoint = curr.getLocation();
		}
		path.addFirst(start.getLocation());
		return path;
	}
	
	private boolean bfsSearch(MapNode start, MapNode goal, 
			HashMap <MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		toExplore.add(start);
		boolean found = false;
		
		// Do the search
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if (curr == goal) {
				found = true;
				break;
			}
			Set<MapEdge> neighbors = curr.getEdges();
			for (MapEdge edge : neighbors){
				GeographicPoint end = edge.getEndLocation();
				MapNode endNode = locationNodeMap.get(end);
				if (!visited.contains(endNode)) {
					visited.add(endNode);
					parentMap.put(endNode, curr);
					toExplore.add(endNode);
				}
			}
		}
		return found;
	}
	
	// to print out the resulting path from bfs() or other search algorithm
	private void printPath(List<GeographicPoint> path) {
		System.out.println("There are " + path.size()+ "Nodes.");
		for (GeographicPoint pt : path) {
			System.out.println(pt);
		}
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Error: null start or goal:  No path exists.");
			return new LinkedList<GeographicPoint>();
			}
		MapNode startNode = locationNodeMap.get(start);
		MapNode goalNode = locationNodeMap.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Error: start or goal not in graph: No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		// initialize the algorithm
		PriorityQueue<MapNode> PQ = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		
		double infDouble = Double.MAX_VALUE;
		
		// set all nodes to infinity
		for(MapNode node : locationNodeMap.values()){
			node.setDistance(infDouble);
			node.setrealDistance(infDouble);
		}
		
		startNode.setrealDistance(0.0);
		startNode.setDistance(0.0);
		
		PQ.add(startNode);
		
		while(!PQ.isEmpty()) {
			MapNode curr = PQ.poll();
			
		    if (!visited.contains(curr)) {
		    	visited.add(curr);
		    	nodeSearched.accept(curr.getLocation());
		    	
		    	HashMap<MapNode, Double> distances = new HashMap<>();
		    	for (MapEdge edge : curr.getEdges()) {
		    		GeographicPoint end = edge.getEndLocation();
					MapNode endNode = locationNodeMap.get(end);
					distances.put(endNode, edge.getDistance());
				}
		    	
				if (curr == goalNode) {
					System.out.println("Number of Visited Nodes: " + visited.size());
					return constructPath(startNode, goalNode, parent);
				}
				else {
					for (MapEdge edge : curr.getEdges()) {
						GeographicPoint end = edge.getEndLocation();
						MapNode endNode = locationNodeMap.get(end);
						if (!visited.contains(endNode)) {
						    double trialDistance = curr.getrealDistance() + distances.get(endNode);
						    if (trialDistance < endNode.getrealDistance()) {
						    	endNode.setDistance(trialDistance);
						    	// this is where we apply the Astar heuristic
						    	// calculate heuristic here and add to trialdistance
						    	endNode.setrealDistance(trialDistance);
						    	parent.put(endNode, curr);
								PQ.offer(endNode);
						    }
						}
					}
				}
		    }
		}
		
		return new LinkedList<GeographicPoint>();
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Error: null start or goal:  No path exists.");
			return new LinkedList<GeographicPoint>();
			}
		MapNode startNode = locationNodeMap.get(start);
		MapNode goalNode = locationNodeMap.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Error: start or goal not in graph: No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		// initialize the algorithm
		PriorityQueue<MapNode> PQ = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		
		double infDouble = Double.MAX_VALUE;
		
		// set all nodes to infinity
		for(MapNode node : locationNodeMap.values()){
			node.setDistance(infDouble);
			node.setrealDistance(infDouble);
		}
		
		startNode.setrealDistance(0.0);
		startNode.setDistance(0.0);
		
		PQ.add(startNode);
		
		while(!PQ.isEmpty()) {
			MapNode curr = PQ.poll();
			
		    if (!visited.contains(curr)) {
		    	visited.add(curr);
		    	nodeSearched.accept(curr.getLocation());
		    	
		    	HashMap<MapNode, Double> distances = new HashMap<>();
		    	for (MapEdge edge : curr.getEdges()) {
		    		GeographicPoint end = edge.getEndLocation();
					MapNode endNode = locationNodeMap.get(end);
					distances.put(endNode, edge.getDistance());
				}
		    	
				if (curr == goalNode) {
					System.out.println("Number of Visited Nodes: " + visited.size());
					return constructPath(startNode, goalNode, parent);
				}
				else {
					for (MapEdge edge : curr.getEdges()) {
						GeographicPoint end = edge.getEndLocation();
						MapNode endNode = locationNodeMap.get(end);
						if (!visited.contains(endNode)) {
						    double trialDistance = curr.getrealDistance() + distances.get(endNode);
						    if (trialDistance < endNode.getrealDistance()) {
						    	endNode.setrealDistance(trialDistance);
						    	// this is where we apply the Astar heuristic
						    	// calculate heuristic here and add to trialdistance
						    	trialDistance += endNode.getLocation().distance(goalNode.getLocation());
						    	endNode.setDistance(trialDistance);
						    	
						    	parent.put(endNode, curr);
								PQ.offer(endNode);
						    }
						}
					}
				}
		    }
		}
		
		return new LinkedList<GeographicPoint>();
	}

	
	
	public static void main(String[] args)
	{
	//	System.out.print("Making a new map...");
		//	MapGraph firstMap = new MapGraph();
		//	System.out.print("DONE. \nLoading the map...");
		//	GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		//	System.out.println("DONE.");
		
		//	GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		//	GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		
		//	List<GeographicPoint> testroute = firstMap.bfs(testStart,testEnd);
		//	firstMap.printPath(testroute);
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/**/
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		simpleTestMap.printPath(testroute);
		simpleTestMap.printPath(testroute2);
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		testMap.printPath(testroute);
		testMap.printPath(testroute2);
		
		// A slightly more complex test using real data

		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		testMap.printPath(testroute);
		testMap.printPath(testroute2);
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*
		 * 	 */
		MapGraph theMap = new MapGraph();
		 
		  
	
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		theMap.printPath(route);
		theMap.printPath(route2);
		
		
	}
	
}
