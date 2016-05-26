package com.github.rinde.rinsim.examples.experiment;

import java.util.LinkedList;
import java.util.LinkedList;
import java.util.List;

import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;

public class RouteChecker {
	// distance between each cross-road
	final double x_crossroadDistance = ExperimentExample.HORIZONTAL_SPACING * ExperimentExample.SPACING;
	final double y_crossroadDistance = ExperimentExample.VERTICAL_SPACING * ExperimentExample.SPACING;
	
	final double secureTime = ExperimentExample.HORIZONTAL_SPACING;
	
	public List<Point> newRoute = new LinkedList<Point>();
	
	public RouteChecker(AgvAgent agv, List<Point> currentRoute, List<Point> agvRoute, RoadModel rm) {
		
		// Counters for the while loop
		int i = 0;
		int j = 0;
		
		while (i < agvRoute.size() && !currentRoute.contains(agvRoute.get(i))) {
			i++;
		}
		
		// if they have a common point find that point and if they happen to pass from
		// the point close enough timewise, change their route
		
		System.out.println(agv.toString().split("@")[1]);
		
		if (i < agvRoute.size()-1) {
			while (!currentRoute.get(j).equals(agvRoute.get(i)) && !isCrossRoad(currentRoute.get(j))) {
				j++;
			}
			
			if (j>0) {
				newRoute = changeRoute(j, currentRoute, rm);
				System.out.println("j LARGER THAN 0");
				//System.out.println(newRoute);
			}
			else {
				newRoute = standAside(j, i, currentRoute, agvRoute);
				System.out.println("j = 000000");
				//System.out.println(newRoute);
			}		
				
		}
		else {
			newRoute = currentRoute;
			System.out.println("NO COLLISION DETECTED");
			//System.out.println(newRoute);
			//System.out.println("MONO ego gamo ton peiraia");
			//System.out.println(newRoute);
		}
		
		// Remove the first point to avoid Exception Throwing, when your the first 
		// point of the path is the same as your current location
		//newRoute.remove(0);
		
	}
	
	public List<Point> changeRoute(int listPos, List<Point> route, RoadModel rm) {
		// list where new route is calculated
		List<Point> changedRoute = new LinkedList<Point>();
		// keep the part of the old route that is not going to change
		List<Point> unchangedRoute = new LinkedList<Point>(route).subList(listPos, route.size()-1);
		
		Point prevCross = prevCrossroad(listPos, route);
		Point newPrevCross =  newPrevCrossroad(listPos, route, prevCross);
		Optional<Point> intermediate = intermediateCrossroad(prevCross, newPrevCross, route.get(listPos));
		
		System.out.println("End");
		System.out.println(route.get(listPos));
		System.out.println("Previous Point");
		System.out.println(route.get(listPos-1));
		System.out.println("prevCross");
		System.out.println(prevCross);
		System.out.println("newPrevCross");
		System.out.println(newPrevCross);
		System.out.println("intermediate");
		System.out.println(intermediate.get());
		
		List <Point> goToPrev = rm.getShortestPathTo(route.get(0), prevCross);
		List <Point> goToIntermediate = rm.getShortestPathTo(prevCross, intermediate.get());
		List <Point> goTonewPrev = rm.getShortestPathTo(intermediate.get(), newPrevCross);
		List <Point> goToEnd = rm.getShortestPathTo(newPrevCross, route.get(listPos));
		
		// Remove end points which are the same with the first from the next list in the row
		goToIntermediate.remove(0);
		goTonewPrev.remove(0);
		goToEnd.remove(0);
		
		// Form a new route to follow
		changedRoute.addAll(goToPrev);
		changedRoute.addAll(goToIntermediate);
		changedRoute.addAll(goTonewPrev);
		changedRoute.addAll(goToEnd);
		changedRoute.addAll(unchangedRoute);
		
		//System.out.println("new route");
		//System.out.println(changedRoute);
		//System.out.println("old route");
		//System.out.println(route);
		
		return changedRoute;
	}
	
	public Point prevCrossroad(int listPos, List<Point> route) {		
		final double xprev = route.get(listPos-1).x;
		final double yprev = route.get(listPos-1).y;
		final double xend = route.get(listPos).x;
		final double yend = route.get(listPos).y;
		
		System.out.println("XPREV");
		System.out.println(xprev);
		System.out.println("XyPREV");
		System.out.println(yprev);
		
		final double xprevCross;
		final double yprevCross;
		
		final double signx = Math.signum(xprev - xend);
		final double signy = Math.signum(yprev - yend);
		
		if (xend%x_crossroadDistance == 0 && yend%y_crossroadDistance == 0) {
			xprevCross = xprev + signx*(x_crossroadDistance - signx*(xprev - xend)%x_crossroadDistance);
			yprevCross = yprev + signy*(y_crossroadDistance - signy*(yprev - yend)%x_crossroadDistance);
		}
		else {
			if (signx >=0) {
				xprevCross = xprev + signx*(x_crossroadDistance - (xprev)%x_crossroadDistance);
			}
			else {
				xprevCross = xprev + signx*(xprev%x_crossroadDistance);
			}
			
			if (signy >=0) {
				yprevCross = yprev + signy*(y_crossroadDistance - (yprev)%y_crossroadDistance);
			}
			else {
				yprevCross = yprev + signy*(yprev%y_crossroadDistance);
			}
			
		}
		
		
		/*System.out.println("what what what");
		System.out.println(xprev - xend);
		System.out.println((xprev - xend)%x_crossroadDistance);
		System.out.println(yprev - yend);
		System.out.println((yprev - yend)%x_crossroadDistance);
		/*System.out.println("xprev");
		System.out.println(xprev);
		System.out.println("yprev");
		System.out.println(yprev);*/
		
		// Create the point
		final Point prev = new Point(xprevCross, yprevCross);
		
		return prev;
	}
	
	public Point newPrevCrossroad(int listPos, List<Point> route,  Point prevCross) {
		final double xend = route.get(listPos).x;
		final double yend = route.get(listPos).y;
		final double xprev = prevCross.x;
		final double yprev = prevCross.y;
		
		// Find the new previous point to change the route of our agv. We randomly choose
		// one of the two possible solutions
		// First find random number between -1 and 1 and then take its sign
		final double r = -1 + 3*Math.random();
		final int signr = (int) Math.signum(r);
		final double xnewPrev = xprev + (signr * Math.signum(xend - xprev + Double.MIN_VALUE)) * x_crossroadDistance;
		final double ynewPrev = yprev + (signr * Math.signum(yend - yprev + Double.MIN_VALUE)) * y_crossroadDistance;
		
		//System.out.println("xnewPrev");
		//System.out.println(xnewPrev);
		//System.out.println("ynewPrev");
		//System.out.println(ynewPrev);
		
		// Check if point is in Graph bounds otherwise take its counterpart
		Point newPrev = InBoundsPoint(new Point(xnewPrev, ynewPrev));
		
		return newPrev;
	}
	
	public Optional<Point> intermediateCrossroad(Point prev, Point newPrev, Point endPoint) {
		Optional<Point> intermediate;
		
		double x1 = (prev.x + newPrev.x + newPrev.y - prev.y)/2;
		double y1 = (prev.y + newPrev.y + prev.x - newPrev.x)/2;
		double x2 = (prev.x + newPrev.x + prev.y - newPrev.y)/2;
		double y2 = (prev.y + newPrev.y + newPrev.x - prev.x)/2;
		
		Point point1 = new Point(x1,y1);
		Point point2 = new Point(x2,y2);
		System.out.println("Intermediate Point 1");
		System.out.println(point1);
		System.out.println("Intermediate Point 2");
		System.out.println(point2);
		
		if (point1.equals(endPoint)) {
			intermediate = Optional.of(point2);
		}
		else {
			intermediate = Optional.of(point1);
		}
		
		return intermediate;
	}
	
	public List<Point> standAside(int currentlistPos, int agvlistPos, List<Point> currentroute, List<Point> agvroute) {
		List<Point> newRoad = new LinkedList<Point>();
		List<Point> pointsaroundCross = new LinkedList<Point>();
		Point movepoint = null;
		
		final Point pointup = new Point(currentroute.get(currentlistPos).x, currentroute.get(currentlistPos).y - 30);
		final Point pointleft = new Point(currentroute.get(currentlistPos).x - 30, currentroute.get(currentlistPos).y);
		final Point pointdown = new Point(currentroute.get(currentlistPos).x, currentroute.get(currentlistPos).y + 30);
		final Point pointright = new Point(currentroute.get(currentlistPos).x + 30, currentroute.get(currentlistPos).y);
		
		pointsaroundCross.add(pointup);
		pointsaroundCross.add(pointleft);
		pointsaroundCross.add(pointdown);
		pointsaroundCross.add(pointright);
		
		for (int i = 0; i < pointsaroundCross.size(); i++) {
			if (!(pointsaroundCross.get(i) == agvroute.get(agvlistPos + 1)) && !(pointsaroundCross.get(i) == agvroute.get(agvlistPos - 1))) {
				movepoint = pointsaroundCross.get(i);
			}
		}
		
		for (int i = 0; i < agvlistPos + 1; i++) {
			newRoad.add(movepoint);
		}
		
		newRoad.addAll(currentroute);
		
		return newRoad;
	}
	
	public boolean isInBounds(Point point) {
		final Point bounds = new Point(ExperimentExample.MAX_WIDTH, ExperimentExample.MAX_HEIGHT);
		final boolean check_x = point.x <= bounds.x;
		final boolean check_y = point.y <= bounds.y;
		final boolean check = check_x &&  check_y;
		
		return check;
	}
	
	public Point InBoundsPoint(Point point) {
		
		Point finalPoint = new Point(0, 0);
		
		final Point pointXout = new Point(ExperimentExample.MAX_WIDTH - x_crossroadDistance, point.y);
		final Point pointYout = new Point(point.x,ExperimentExample.MAX_HEIGHT - y_crossroadDistance);
		
		if (isInBounds(point)){
			finalPoint = point;
		}
		else{
			if (point.x > ExperimentExample.MAX_WIDTH) {
				finalPoint = pointXout;
			}
			if (point.y > ExperimentExample.MAX_HEIGHT) {
				finalPoint = pointYout;
			}
		}
		
		return finalPoint;
	}
	
	public boolean isCrossRoad(Point point) {
		boolean isCross = (point.x%480 == 0 && point.y%480 == 0);
		
		return isCross;
	}

}
