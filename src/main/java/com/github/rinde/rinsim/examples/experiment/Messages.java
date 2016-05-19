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

public class Messages {

	Optional<List<List<Point>>> routingInfo ;
	Optional<List<Point>> senderRoute ;
	Optional<List<List<Point>>> timeStamps ;
	Optional<List<String>> aGV_IDs ;
	Optional<String> senderID;
	
	public Messages(){
		routingInfo = Optional.absent();
		senderRoute = Optional.absent();
		timeStamps = Optional.absent();
		aGV_IDs = Optional.absent();
		senderID = Optional.absent();
	}
	
	public void setSenderRoute(List<Point> Route){
		senderRoute = Optional.of(Route);
	}
	
	public void setSenderID(String ID){
		senderID = Optional.of(ID);
	}
	
	public List<Point> getSenderRoute(){
		List<Point> route = senderRoute.get();
		return route;
	}
	
	public String getSenderID(){
		String ID = senderID.get();
		return ID;
	}
	
}
