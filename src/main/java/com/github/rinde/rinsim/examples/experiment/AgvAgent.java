package com.github.rinde.rinsim.examples.experiment;

import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;

public class AgvAgent extends Vehicle{

	public AgvAgent(VehicleDTO dto){
		super(dto);
		//super(dto).builder().startPosition(initPosi);
	}

	@Override
	protected void tickImpl(TimeLapse time) {
		// TODO Auto-generated method stub
		
	}
}
