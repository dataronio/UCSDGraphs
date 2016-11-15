/**
 * @author D.W. Shimer
 * 
 * A class which represents an Edge in a MapGraph
 * Edges are one way roads from Start to End Nodes 
 *
 */
package roadgraph;

import geography.GeographicPoint;

class MapEdge {
	// a MapEdge has a start GeographicPoint location and an end location
	private GeographicPoint start;
	private GeographicPoint end;
	
	// a MapEdge has a roadName and roadType strings
    private String roadName;
    private String roadType;
    
    // a MapEdge has a double distance length from start to end
    private double distance;
    
    
    // constructor for MapEdge
    MapEdge(GeographicPoint start, GeographicPoint end, String roadName, 
    		String roadType, double distance) {
    	this.start = start;
    	this.end = end;
    	this.roadName = roadName;
    	this.roadType = roadType;
    	this.distance = distance;
    }
    
    // get the location of the start Node
    GeographicPoint getStartLocation() {
    	return this.start;
    }
    
    // get the location of the end Node
    GeographicPoint getEndLocation() {
    	return this.end;
    }
    
    // get the roadName
    String getRoadName() {
    	return this.roadName;
    }
    
    // get the roadType
    String getRoadType() {
    	return this.roadType;
    }
    
 // get the distance of the road
    double getDistance() {
    	return this.distance;
    }
    
}
