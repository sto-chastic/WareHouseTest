package com.github.rinde.rinsim.examples.experiment;

import java.util.LinkedList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.Model.AbstractModel;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.Message;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.PDPObject;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.GraphRoadModel;
import com.github.rinde.rinsim.core.model.road.MovingRoadUser;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModels;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.MessageContents;
import com.github.rinde.rinsim.geom.ConnectionData;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.pdptw.common.AddVehicleEvent;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;
import com.github.rinde.rinsim.examples.experiment.Mssgs;


public class AgvAgent extends Vehicle implements MovingRoadUser, CommUser {

	static final double MIN_RANGE = .2;
	static final double MAX_RANGE = 1.5;
	static final long LONELINESS_THRESHOLD = 10 * 1000;
	
	Optional<RoadModel> roadModel;
	Optional<CommDevice> device;
	Optional<Point> destination;
	Optional <Point> endPoint;
	long lastReceiveTime;
	private final double range;
	private final double reliability;
	private final double rng;
	
	private static final double SPEED = 1000d;
	private Optional<Parcel> curr;
	
	public static List<String> AGV_id = new LinkedList<String>();
	
	//List<List<Point>> route = new List<List<Point>>(3);
	
	//LinkedList<List<Point>> routingInfo = new LinkedList<List<Point>>(3);
	
	// the path to follow and the path of another agv sent to this one
	public Optional<List<Point>> pathToFollow;
    public Optional<List<Point>> agvRoute;
	
	// Optional checking of route for collision with another agv
	private Optional<List<Point>> checkedRoute;
	
	// LinkedList that will act as queue for our followPath() method
	private LinkedList<Point> queue_checkedRoute = new LinkedList<>();
	
	public AgvAgent(VehicleDTO dto){
		super(dto);
		curr = Optional.absent();
		checkedRoute = Optional.absent();
		pathToFollow = Optional.absent();
		agvRoute = Optional.absent();
		
		rng = 1000;
	    device = Optional.absent();
	    //roadModel = Optional.absent();
	    destination = Optional.absent();
	    
	    endPoint = Optional.absent();

	    range = MIN_RANGE + rng * (MAX_RANGE - MIN_RANGE);
	    reliability = 0.9;
	}
	

	@Override
	public void setCommDevice(CommDeviceBuilder builder) {
	    if (range >= 0) {
	        builder.setMaxRange(range);
	      }
	      device = Optional.of(builder
	        .setReliability(reliability)
	        .build());
	}
	
	public AgvAgent getThisAgvID() {
		final RoadModel rm = getRoadModel();
		AgvAgent thisAgv = null;
		//--------Give IDs to all AGVs------
	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    //AGV_id = existingAgvs;
	    for (int i=0; i<rm.getObjectsOfType(AgvAgent.class).size(); i++){
	    	if (i == rm.getObjectsOfType(AgvAgent.class).size() - 1) {
	    		AGV_id.add(existingAgvs.get(i).toString().split("@")[1]);
	    	}
	    }

	    for (int i = 0; i < rm.getObjectsOfType(AgvAgent.class).size(); i++) {
	    	if (rm.getPosition(existingAgvs.get(i)).equals(this.getPosition().get())) {
	    		thisAgv = existingAgvs.get(i);
	    	}
	    }
		return thisAgv;
	}
	
	public void getAvgId(){
		final RoadModel rm = getRoadModel();
		//--------Give IDs to all AGVs------
	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    //AGV_id = existingAgvs;
	    for (int i=0; i<rm.getObjectsOfType(AgvAgent.class).size(); i++){
	    	if (i == rm.getObjectsOfType(AgvAgent.class).size() - 1) {
	    		AGV_id.add(existingAgvs.get(i).toString().split("@")[1]);
	    	}
	    }
	    System.out.println(AGV_id);
	}
	
	
	@Override
	protected void tickImpl(TimeLapse time) {
		final RoadModel rm = getRoadModel();
	    final PDPModel pm = getPDPModel();
	    
	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    
	    //--------Package Handling---------- 
	   
	    if (!time.hasTimeLeft()) {
	        return;
	    }
	    if (!curr.isPresent()) {
	      curr = Optional.fromNullable(RoadModels.findClosestObject(
	        rm.getPosition(this), rm, Parcel.class));
	    }
	    
	    if (destination.isPresent()) {
	    	endPoint = destination;
	    }
	      
	    if (curr.isPresent()) {
	        final boolean inCargo = pm.containerContains(this, curr.get());
	        // sanity check: if it is not in our cargo AND it is also not on the
	        // RoadModel, we cannot go to curr anymore.
	        if (!inCargo && !rm.containsObject(curr.get())) {
	          curr = Optional.absent();
	        } else if (inCargo) {
	          // if it is in cargo, go to its destination by checking for collision if
	          // there is a message from the other AGVs around it.
	        	destination = Optional.of(curr.get().getDeliveryLocation());
	        	
	        	if (checkedRoute.isPresent() && !queue_checkedRoute.isEmpty()) {
	        		// follow the path calculated
	        		//System.out.println("Checked Route");
	        		//if (!queue_checkedRoute.equals(checkedRoute.get())) {
	        			//System.out.println(getThisAgvID().toString().split("@")[1]);
	        			//System.out.println(checkedRoute.get());
	        			//System.out.println(queue_checkedRoute);
	        			//System.out.println(this.getPosition().get());
	        			//System.out.println(rm.getShortestPathTo(this, curr.get().getDeliveryLocation()));
	        			//System.out.println(pathToFollow.get());
	        			//System.out.println(agvRoute.get());
	        		//}
	        			// save the destination of the agv
	        			//endPoint = Optional.of(queue_checkedRoute.getLast());
	        		
	        		//System.out.println("IS EMPTY OR IS NOT EMPTY");
	        		//System.out.println(!checkedRoute.get().isEmpty());
	        			
	        		// check if the distance between this point and the next is an exact quantized
	        		// distance and correct the mini error
	        		LinkedList<Point> thisToEnd = new LinkedList<Point>(rm.getShortestPathTo(this, endPoint.get()));
	        		boolean checkx = queue_checkedRoute.getFirst().x - thisToEnd.getFirst().x == 30;
	        		boolean checky = queue_checkedRoute.getFirst().y - thisToEnd.getFirst().y == 30;	
	        		if (checkx || checky) {
	        			queue_checkedRoute.addFirst(new Point(thisToEnd.getFirst().x, thisToEnd.getFirst().y));
	        		}
	        		
	        		rm.followPath(this,queue_checkedRoute, time);
	        		
	        		//System.out.println("Checked Route After");
	        		//System.out.println(queue_checkedRoute);
	        	}
	        	else {
	        		// Just casually move to the designated location
	        		//System.out.println("Shortest Route");
	        		//System.out.println(rm.getShortestPathTo(this, curr.get().getDeliveryLocation()));
	  	      		rm.moveTo(this, curr.get().getDeliveryLocation(), time);
	  	      		//rm.followPath(this, new LinkedList<Point>(rm.getShortestPathTo(this, curr.get().getDeliveryLocation())), time);
	  	      		// save the new destination of the agv
	  	      		endPoint = Optional.of(curr.get().getDeliveryLocation());
	  	      		System.out.println("ENTERED IN THIS LOOP");
	        	}
            if (rm.getPosition(this).equals(curr.get().getDeliveryLocation())) {
            // deliver when we arrive
	             pm.deliver(this, curr.get(), time);
	        }
            } else {
            	if (checkedRoute.isPresent() && !queue_checkedRoute.isEmpty()) {
            		rm.followPath(this,queue_checkedRoute, time);
            	}
            	else {
            		// if not communication with other AGV go there as soon as possible
                	rm.moveTo(this, curr.get(), time);
            	}
            	// make sure that destination has no value
            	destination = Optional.absent();
            	// save the destination of the AGV until it picks the package
            	endPoint = Optional.of(curr.get().getPickupLocation());
            	//System.out.println(getThisAgvID().toString().split("@")[1]);
    			//System.out.println(endPoint);
    			//System.out.println(queue_checkedRoute);
	        if (rm.equalPosition(this, curr.get())) {
	            // pickup package
	        	pm.pickup(this, curr.get(), time);
	        }
	          }
	      }
	    
	    // Path to follow
	    if (endPoint.isPresent()) {
	    	pathToFollow = Optional.of(rm.getShortestPathTo(this, endPoint.get()));
	    	//System.out.println(getThisAgvID().toString().split("@")[1]);
	    	//System.out.println(pathToFollow.get());
	    }
	      
	    //--------Message Handling----------
	    
		if (device.get().getUnreadCount() > 0){
	    	lastReceiveTime = time.getStartTime();
		    
		    // device updates its information about the other AGVs
		    ImmutableList<Message> M = device.get().getUnreadMessages();
	    	
		    for (int i = 0; i< M.size(); i++){
		    	Mssgs temp = (Mssgs) M.get(i).getContents();
		    	
		    	agvRoute = temp.getSenderRoute();
		    	
		    	System.out.println(getThisAgvID().toString().split("@")[1]);
		    	
		    	System.out.println("Message Received ROUTE SENT");
		    	System.out.println(agvRoute.isPresent());
		    	
		    	System.out.println("PATH FOUND");
		    	System.out.println(pathToFollow.isPresent());
		    	
		    	if (agvRoute.isPresent() && pathToFollow.isPresent()) {
		    		// create an instance of the RouteChecker class which is responsible for comparing
		    		// the 2 paths and it returns a new one, changed or not according to if a collision
		    		// would be possible
		    		
		    		//Initialize the queue each time to avoid elements aggregation
		    		queue_checkedRoute.removeAll(queue_checkedRoute);
		    		RouteChecker check = new RouteChecker(getThisAgvID(), pathToFollow.get(), agvRoute.get(), rm);
		    		checkedRoute = Optional.of(check.newRoute);
		    		queue_checkedRoute.addAll(checkedRoute.get());
		    		System.out.println("CALCULATION OF CHECKEDROUTE");
		    		//System.out.println(getThisAgvID().toString().split("@")[1]);
		    		System.out.println(checkedRoute.get());
	    			//System.out.println(pathToFollow.get());
	    			//System.out.println(agvRoute.get());
		    		//System.out.println("CALCULATION OF CHECKEDROUTE");
		    	}
		    }
		    
	    }
		else if (device.get().getUnreadCount() == 0) {
			//Create new Mssgs instanse for sending or broadcasting
			Mssgs newMsg = new Mssgs();		
    		// device broadcasts the route that it plans to follow
			if (endPoint.isPresent()) {
				newMsg.setSenderID(this.toString().split("@")[1]);
    			newMsg.setSenderRoute(rm.getShortestPathTo(this, endPoint.get()));
    			device.get().broadcast(newMsg);
			}
			
			// make sure that 
    		
    	} 
/*		else if (time.getStartTime() - lastReceiveTime > LONELINESS_THRESHOLD) {
			device.get().broadcast(Messages.WHERE_IS_EVERYBODY);
		}	*/      
	      	
	}
	
	@Override
	public Optional<Point> getPosition() {
		final RoadModel rm = getRoadModel();
		if (rm.containsObject(this)) {
		    return Optional.of(rm.getPosition(this));
		  }
    return Optional.absent();
	}


}