package com.github.rinde.rinsim.examples.experiment;

import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
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
import com.google.common.base.Optional;


public class AgvAgent extends Vehicle implements MovingRoadUser, CommUser{

	static final double MIN_RANGE = .2;
	static final double MAX_RANGE = 1.5;
	static final long LONELINESS_THRESHOLD = 10 * 1000;
	
	//Optional<RoadModel> roadModel;
	Optional<CommDevice> device;
	Optional<Point> destination;
	long lastReceiveTime;
	private final double range;
	private final double reliability;
	private final double rng;
	
	private static final double SPEED = 1000d;
	private Optional<Parcel> curr;
	
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
	
	
	@Override
	protected void tickImpl(TimeLapse time) {
		final RoadModel rm = getRoadModel();
	    final PDPModel pm = getPDPModel();
	    
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
            rm.moveTo(this, curr.get().getDeliveryLocation(), time);
            if (rm.getPosition(this).equals(curr.get().getDeliveryLocation())) {
            // deliver when we arrive
	          	/*System.out.print("Delivery");
	           	System.out.print(curr.get().getDeliveryLocation());
	           	System.out.print("/Delivery");*/
	             pm.deliver(this, curr.get(), time);
	        }
            } else {
	           // it is still available, go there as fast as possible
            rm.moveTo(this, curr.get(), time);
	            /*System.out.print("PickUp");
	            System.out.print(curr.get().getPickupLocation());
	            System.out.print("/PickUp");*/
	            if (rm.equalPosition(this, curr.get())) {
	              // pickup package
	              pm.pickup(this, curr.get(), time);
	              
	            }
	          }
	      }
	      
	    //--------Message Handling----------

	   
		if (device.get().getUnreadCount() > 0){
	    	lastReceiveTime = time.getStartTime();
		    device.get().getUnreadMessages();
		    device.get().broadcast(Messages.NICE_TO_MEET_YOU);
	    } 
		else if (device.get().getReceivedCount() == 0) {
    		device.get().broadcast(Messages.HELLO_WORLD);
    	} 
		else if (time.getStartTime() - lastReceiveTime > LONELINESS_THRESHOLD) {
			device.get().broadcast(Messages.WHERE_IS_EVERYBODY);
		}	      
	      	
	}
	
    enum Messages implements MessageContents {
	    HELLO_WORLD, NICE_TO_MEET_YOU, WHERE_IS_EVERYBODY;
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
