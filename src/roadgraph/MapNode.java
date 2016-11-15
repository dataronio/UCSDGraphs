/**
 * @author D.W. Shimer
 * 
 * A class which represents a Node of a geographic location
 * Nodes exist in a MapGraph and have MapEdges to other Nodes
 *
 */
package roadgraph;

import java.util.HashSet;
import java.util.Set;
import geography.GeographicPoint;

class MapNode implements Comparable<MapNode> {
	private GeographicPoint location;
    private HashSet <MapEdge> neighbors;  // Set of edges that lead out of node
    double distance; //realDistance + heuristicDistance
    double realDistance;
    
    // constructor just specify a GeographicPoint location
    MapNode(GeographicPoint location) {
    	this.location = location;
    	this.neighbors = new HashSet<MapEdge>();
    	this.distance = 0.0;
		this.realDistance = 0.0;
    }
    
 // constructor just specify a GeographicPoint location and distance
    MapNode(GeographicPoint location, double distance) {
    	this.location = location;
    	this.neighbors = new HashSet<MapEdge>();
    	this.distance = distance;
		this.realDistance = 0.0;
    }
    
 // constructor just specify a GeographicPoint location and distance and heurisicDistance
    MapNode(GeographicPoint location, double distance, double heuristicDistance) {
    	this.location = location;
    	this.neighbors = new HashSet<MapEdge>();
    	this.distance = distance;
		this.realDistance = heuristicDistance;
    }
    
    /**
     * 
     * @param edge  A road from this node to another
	 * @return true if a edge was added, false if it was not
     */
    boolean addEdge(MapEdge edge) {
    	return neighbors.add(edge);
    }
    
    /**
     * 
	 * @return Set <MapEdge>
     */
    Set <MapEdge> getEdges() {
    	return neighbors;
    }
    
    
    /**
     * 
	 * @return int number of roads out of this node
     */
    int getNumEdges() {
    	return neighbors.size();
    }
    
    public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
    /**
     * getter for the location
     * 
	 * @return GeographicPoint
     */
    GeographicPoint getLocation() {
    	return this.location;
    }
    
     // set the node distance 
 	public void setDistance(double distance) {
 	    this.distance = distance;
 	}
    
    public Double getDistance() {
        return this.distance; // + heuristicDistance;
    }
    
    public Double getrealDistance() {
        return this.realDistance;
    }
    
       // set node realDistance 
 	public void setrealDistance(double distance) {
 	    this.realDistance = distance;
 	}
    
    @Override
    public int compareTo(MapNode o) {
        MapNode Node = (MapNode) o;
        return (this.getDistance().compareTo(Node.getDistance()));
    }
}
