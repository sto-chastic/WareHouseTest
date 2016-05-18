package com.github.rinde.rinsim.examples.experiment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

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
import com.github.rinde.rinsim.core.model.road.MovingRoadUser;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModels;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.MessageContents;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.pdptw.common.AddVehicleEvent;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;


public class AgvAgent extends Vehicle implements MovingRoadUser, CommUser{

	static final double MIN_RANGE = .2;
	static final double MAX_RANGE = 1.5;
	static final long LONELINESS_THRESHOLD = 10 * 1000;
	
	Optional<RoadModel> roadModel;
	Optional<CommDevice> device;
	Optional<Point> destination;
	long lastReceiveTime;
	private final double range;
	private final double reliability;
	private final double rng;
	
	private static final double SPEED = 1000d;
	private Optional<Parcel> curr;
	
	public static ArrayList<String> AGV_id = new ArrayList<String>();
	
	//public static ArrayList<Point> route = new ArrayList<Point>();
	ArrayList<List<Point>> route = new ArrayList<List<Point>>(3);
	
	//public static ArrayList<Point> sent_coords;
	
	ArrayList<List<Point>> routingInfo = new ArrayList<List<Point>>(3);
	
	public AgvAgent(VehicleDTO dto){
		super(dto);
		curr = Optional.absent();
		
		rng = 1000;
	    device = Optional.absent();
	    //roadModel = Optional.absent();
	    destination = Optional.absent();

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
	
	public void getAvgId(){
		final RoadModel rm = getRoadModel();
		//--------Give IDs to all AGVs------
	    final ArrayList<AgvAgent> existingAgvs = new ArrayList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
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
	    
	    final ArrayList<AgvAgent> existingAgvs = new ArrayList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    
	    //--------Package Handling---------- 
	   
	    if (!time.hasTimeLeft()) {
	        return;
	    }
	    if (!curr.isPresent()) {
	      curr = Optional.fromNullable(RoadModels.findClosestObject(
	        rm.getPosition(this), rm, Parcel.class));
	    }
	      
	    if (curr.isPresent()) {
	        final boolean inCargo = pm.containerContains(this, curr.get());
	        // sanity check: if it is not in our cargo AND it is also not on the
	        // RoadModel, we cannot go to curr anymore.
	        if (!inCargo && !rm.containsObject(curr.get())) {
	          curr = Optional.absent();
	        } else if (inCargo) {
	          // if it is in cargo, go to its destination
	        	destination = Optional.of(curr.get().getDeliveryLocation());
	  	      	rm.moveTo(this, curr.get().getDeliveryLocation(), time);
            if (rm.getPosition(this).equals(curr.get().getDeliveryLocation())) {
            // deliver when we arrive
	             pm.deliver(this, curr.get(), time);
	        }
            } else {
	           // it is still available, go there as fast as possible
            rm.moveTo(this, curr.get(), time);
	            if (rm.equalPosition(this, curr.get())) {
	              // pickup package
	              pm.pickup(this, curr.get(), time);
	              
	            }
	          }
	      }
	      
	    //--------Message Handling----------
	    
		if (device.get().getUnreadCount() > 0){
	    	lastReceiveTime = time.getStartTime();
		    
		    //device.get().broadcast(Messages.NICE_TO_MEET_YOU);
		    
		    // device updates its information about the other AGVs
		    ImmutableList<Message> M = device.get().getUnreadMessages();
		    
		  //Initialize routingInfo
	    	for (int i = 0; i< 3; i++){
	    		routingInfo.add(i, Arrays.asList(new Point(0,0),new Point(0,0)));
	    	}
	    	
		    for (int i = 0; i< M.size(); i++){
		    	if (M.get(i).getContents() == Msgs.AGV_ROUTING){
		    		CommUser sender = M.get(i).getSender();
		    		for (int j = 0; j < AGV_id.size(); j++){
		    			routingInfo.set(j, route.get(j));
		    		}
		    	}
		    }
		    System.out.println(routingInfo.get(0));
		    System.out.println(routingInfo.get(1));
		    System.out.println(routingInfo.get(2));
		 
	    } 
		else if (device.get().getReceivedCount() == 0) {
    		//device.get().broadcast(Messages.HELLO_WORLD);
    		
    		// device broadcasts the route that it plans to follow
    		if (destination.isPresent()){    			
        		for (int i=0; i<=existingAgvs.size()-1; i++){
        			if(this!=existingAgvs.get(i)){
        				route.add(i, rm.getShortestPathTo(this, destination.get()));
        				device.get().send(Msgs.AGV_ROUTING, existingAgvs.get(i));
        			}
        			else{
        				// make sure that there is always a value
        			    route.add(i, Arrays.asList(new Point(0,0),new Point(0,0)));
        			}
        		}
    		}
    		else{
    			for (int i = 0; i<=existingAgvs.size()-1; i++){
    	    		route.add(i, Arrays.asList(new Point(0,0),new Point(0,0)));
    	    	}
    		}
    		
    	} 
		else if (time.getStartTime() - lastReceiveTime > LONELINESS_THRESHOLD) {
			device.get().broadcast(Messages.WHERE_IS_EVERYBODY);
		}	      
	      	
	}
	
	enum Messages implements MessageContents {
	    HELLO_WORLD, NICE_TO_MEET_YOU, WHERE_IS_EVERYBODY, M1;
    	}
	
	enum Msgs implements MessageContents{
		AGV_ROUTING;
		Msgs(){
		}
	};

	@Override
	public Optional<Point> getPosition() {
		final RoadModel rm = getRoadModel();
		if (rm.containsObject(this)) {
		    return Optional.of(rm.getPosition(this));
		  }
    return Optional.absent();
	}


}
