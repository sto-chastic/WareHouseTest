package com.github.rinde.rinsim.examples.demo.factory;

import static com.google.common.collect.Lists.newArrayList;
import static java.util.Arrays.asList;

import java.math.RoundingMode;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nullable;
import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import org.apache.commons.math3.random.MersenneTwister;
import org.apache.commons.math3.random.RandomGenerator;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Monitor;

import com.github.rinde.rinsim.core.Simulator;
import com.github.rinde.rinsim.core.model.pdp.DefaultPDPModel;
import com.github.rinde.rinsim.core.model.time.TickListener;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.event.Listener;
import com.github.rinde.rinsim.examples.demo.swarm.SwarmDemo;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.Graphs;
import com.github.rinde.rinsim.geom.LengthData;
import com.github.rinde.rinsim.geom.MultimapGraph;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.ui.View;
import com.github.rinde.rinsim.ui.renderers.GraphRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.RoadUserRenderer;
import com.google.common.collect.ImmutableList;
import com.google.common.math.DoubleMath;

import org.apache.commons.math3.random.RandomGenerator;
import com.github.rinde.rinsim.core.Simulator;
import com.github.rinde.rinsim.core.model.road.MovingRoadUser;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders;
import com.github.rinde.rinsim.core.model.time.TickListener;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.ui.View;
import com.github.rinde.rinsim.ui.renderers.PlaneRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.RoadUserRenderer;

import com.github.rinde.rinsim.examples.demo.factory.*;
/**
* @author Rinde van Lon
*
*/
public final class FactoryExample {

 static final double POINT_DISTANCE = 5d;
 static final long SERVICE_DURATION = 120000;
 static final double AGV_SPEED = 10;
 static final int CANVAS_MARGIN = 30;

 static final int FONT_SIZE = 10;

 static final int VERTICAL_SPACING = 16;
 static final int HORIZONTAL_SPACING = 16;
 static final int HEIGHT = 160;
 static final int WIDTH = 160;
 static final int NUM_VEHICLES = 12;
 static final int FULL_HD_W = 1920;
 static final int SPEED_UP = 4;
 static final long RANDOM_SEED = 123L;
 
 static final Point Ex_1 = new Point(0,0);

 // spacing between text pixels
 static final double SPACING = 30d;//Granularity of movement

 private FactoryExample() {}

 /**
  * Starts the example.
  * @param args One optional argument specifying the simulation end time is
  *          supported.
  */
 public static void main(@Nullable String[] args) {
   final long endTime = args != null && args.length >= 1 ? Long
     .parseLong(args[0]) : Long.MAX_VALUE;

   final Display d = new Display();
   @Nullable
   Monitor sec = null;
   for (final Monitor m : d.getMonitors()) {
     if (d.getPrimaryMonitor() != m) {
       sec = m;
       break;
     }
   }
   run(endTime, d, sec, null);
 }

 /**
  * Run the example.
  * @param endTime The time to stop.
  * @param display The display to show it on.
  * @param m The monitor to show it on.
  * @param list Listener for events.
  * @return The simulator used in the example.
  */
 public static Simulator run(final long endTime, Display display,
     @Nullable Monitor m, @Nullable Listener list) {

   final Rectangle rect;
   if (m != null) {
     if (list != null) {
       rect = m.getClientArea();
     } else {
       // full screen
       rect = m.getBounds();
     }
   } else {
     rect = display.getPrimaryMonitor().getClientArea();
   }
   /*
   private static boxes(){
	   for (int i = 0; i < NUM_BOXES; i++) {
		   final ImmutableList<Point> pointy2 = new ImmutableList.Builder<Point>()
			       .add(new Point(0.0,0.0))
			       .build();
	   }
			 
		return 
	}
   */
   
   final ImmutableList<Point> pointy2 = new ImmutableList.Builder<Point>()
	       .add(new Point(0.0,0.0))
	       .build();

   final ImmutableList<ImmutableList<Point>> pointy = new ImmutableList.Builder<ImmutableList<Point>>()
       .add(pointy2)
       .build();
   
//   final ImmutableList<ImmutableList<Point>> points = createPoints(l);


   View.Builder view = View.builder()
     .with(GraphRoadModelRenderer.builder()
       .withMargin(CANVAS_MARGIN))
     .with(BoxRenderer.builder())
     .with(
       RoadUserRenderer.builder()
         .withImageAssociation(AGV.class,
           "/graphics/flat/forklift2.png"))
     .withTitleAppendix("Factory Demo")
     .withAutoPlay()
     .withAutoClose()
     .withSpeedUp(SPEED_UP);

   if (m != null) {
     view = view.withMonitor(m)
       //.withResolution(m.getClientArea().width, m.getClientArea().height)
       .withResolution(1800, 720)
       .withDisplay(display);

     if (list != null) {
       view = view.withCallback(list)
         .withAsync();
     } else {
       //view = view.withFullScreen();
     }
   }

   final RandomGenerator rng = new MersenneTwister(RANDOM_SEED);
   final Simulator simulator = Simulator
     .builder()
     .setRandomGenerator(rng)
     .addModel(RoadModelBuilders.staticGraph(
    		 createGrid(WIDTH, HEIGHT, VERTICAL_SPACING, HORIZONTAL_SPACING, SPACING)))
     .addModel(
       DefaultPDPModel.builder())
     .addModel(AgvModel.builder().withPoints(pointy, pointy2))    
     .addModel(view)
     .build();
   
   final RoadModel roadModel = simulator.getModelProvider().getModel(
		      RoadModel.class);

   for (int i = 0; i < NUM_VEHICLES; i++) {
     simulator.register(new AGV(roadModel.getRandomPosition(rng)));
   }

   simulator.addTickListener(new TickListener() {
     public void tick(TimeLapse time) {
       if (time.getStartTime() > endTime) {
         simulator.stop();
       }
     }

     public void afterTick(TimeLapse timeLapse) {}
   });

   simulator.start();
   return simulator;
 }


/*
 private static ImmutableList<ImmutableList<Point>> createPoints(
		 Iterable<Point> l) {
	final ImmutableList.Builder<ImmutableList<Point>> pointBuilder =
			ImmutableList.builder();
		    //for (final ImmutableList<Point> p : l) {
		      pointBuilder.add(l);
		    //}
	return pointBuilder.build();
}*/

static Graph<LengthData> createGrid(int width, int height, int hLines,
     int vLines, double distance) {
   final Graph<LengthData> graph = new MultimapGraph<LengthData>();

   int v = 0;
   // draw vertical lines
   for (int i = 0; i < width + 1; i++) {
     Point prev = new Point(i * distance, 0);
     if (i % vLines == 0) {
       for (int j = 1; j < height; j++) {
         final Point cur = new Point(i * distance, j * distance);
         if (v % 2 == 0) {
           graph.addConnection(prev, cur);
         } else {
           graph.addConnection(cur, prev);
         }
         prev = cur;
       }
       v++;
     }
   }

   int y = 1;
   for (int i = 0; i < height; i++) {
     Point prev = new Point(0, i * distance);
     if (i % hLines == 0) {
       for (int j = 1; j < width + 1; j++) {
         final Point cur = new Point(j * distance, i * distance);
         if (y % 2 == 0) {
           graph.addConnection(prev, cur);
         } else {
           graph.addConnection(cur, prev);
         }
         prev = cur;
       }
     }
     y++;
   }
   return graph;
 }
}
