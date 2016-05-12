package com.github.rinde.rinsim.examples.experiment;

import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModels;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;


public class AgvAgent extends Vehicle{

	private static final double SPEED = 1000d;
	private Optional<Parcel> curr;
	
	public AgvAgent(VehicleDTO dto){
		super(dto);
		curr = Optional.absent();
	}

	@Override
	protected void tickImpl(TimeLapse time) {
		final RoadModel rm = getRoadModel();
	    final PDPModel pm = getPDPModel();
	    
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
		
	}
}
