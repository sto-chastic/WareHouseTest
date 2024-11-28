package com.github.rinde.rinsim.examples.experiment;

// Import statements for various functionalities used in the program
import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.collect.Lists.newArrayList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import javax.measure.unit.SI;
import org.apache.commons.math3.random.MersenneTwister;
import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.SimulatorAPI;
import com.github.rinde.rinsim.core.model.ModelBuilder;
import com.github.rinde.rinsim.core.model.comm.CommModel;
import com.github.rinde.rinsim.core.model.pdp.DefaultPDPModel;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.TimeWindowPolicy.TimeWindowPolicies;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.CollisionGraphRoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders.CollisionGraphRMB;
import com.github.rinde.rinsim.experiment.Experiment;
import com.github.rinde.rinsim.experiment.Experiment.SimulationResult;
import com.github.rinde.rinsim.experiment.ExperimentResults;
import com.github.rinde.rinsim.experiment.MASConfiguration;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.LengthData;
import com.github.rinde.rinsim.geom.ListenableGraph;
import com.github.rinde.rinsim.geom.MultimapGraph;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.pdptw.common.AddDepotEvent;
import com.github.rinde.rinsim.pdptw.common.AddParcelEvent;
import com.github.rinde.rinsim.pdptw.common.AddVehicleEvent;
import com.github.rinde.rinsim.pdptw.common.StatsStopConditions;
import com.github.rinde.rinsim.pdptw.common.TimeLinePanel;
import com.github.rinde.rinsim.scenario.Scenario;
import com.github.rinde.rinsim.scenario.StopConditions;
import com.github.rinde.rinsim.scenario.TimeOutEvent;
import com.github.rinde.rinsim.scenario.TimedEvent;
import com.github.rinde.rinsim.scenario.TimedEventHandler;
import com.github.rinde.rinsim.ui.View;
import com.github.rinde.rinsim.ui.renderers.CommRenderer;
import com.github.rinde.rinsim.ui.renderers.GraphRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.PDPModelRenderer;
import com.github.rinde.rinsim.ui.renderers.PlaneRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.Renderer;
import com.github.rinde.rinsim.ui.renderers.RoadUserRenderer;
import com.github.rinde.rinsim.util.TimeWindow;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;

/**
 * Example demonstrating how to use the Experiment class in RinSim.
 * Defines and runs experiments involving scenarios and configurations.
 */
public final class ExperimentExample {
  
  // Constants for simulation configuration
  private static final Point RESOLUTION = new Point(300, 300);
  private static final double VEHICLE_SPEED_KMH = 60d;

  private static final long M1 = 60 * 1000L;  // 1 minute in milliseconds
  private static final long M4 = 4 * 60 * 1000L; // 4 minutes in milliseconds
  private static final long M20 = 20 * 60 * 1000L; // 20 minutes in milliseconds
  private static final long M30 = 30 * 60 * 1000L; // 30 minutes in milliseconds
  private static final long EXP_TIME_DURATION = 10 * 60 * 60 * 1000L; // 10 hours

  // Grid and simulation settings
  static final int VERTICAL_SPACING = 16;
  static final int HORIZONTAL_SPACING = 16;
  static final int HEIGHT = 160;
  static final int WIDTH = 160;

  static final double SPACING = 30d;
  static final double MAX_HEIGHT = HEIGHT * SPACING;
  static final double MAX_WIDTH = WIDTH * SPACING;
  static final double MIN_VER = VERTICAL_SPACING * SPACING;
  static final double MIN_HOR = HORIZONTAL_SPACING * SPACING;

  static final int VEHICLES_NUMBER = 4;  // Number of vehicles
  static final int PARCELS_NUMBER = 10;  // Number of parcels
  static final int DEPOT_NUMBER = 3;  // Number of depots

  static final long RANDOM_SEED = 53L;
  private static final double VEHICLE_LENGTH = 2d;
  
  // List of depot locations
  static ArrayList<Point> point_dep_list = new ArrayList<>(DEPOT_NUMBER);

  private ExperimentExample() {}

  /**
   * Main method to run the simulation experiment.
   * Handles command-line arguments for customization.
   */
  public static void main(String[] args) {
    int uiSpeedUp = 1;
    final int index = Arrays.binarySearch(args, "speedup");

    // Parse and handle the "speedup" argument
    String[] arguments = args;
    if (index >= 0) {
      checkArgument(arguments.length > index + 1,
        "speedup option requires an integer indicating the speedup.");
      uiSpeedUp = Integer.parseInt(arguments[index + 1]);
      checkArgument(uiSpeedUp > 0, "speedup must be a positive integer.");
      final List<String> list = new ArrayList<>(Arrays.asList(arguments));
      list.remove(index + 1);
      list.remove(index);
      arguments = list.toArray(new String[] {});
    }

    final Optional<ExperimentResults> results;

    // Build and configure the experiment
    results = Experiment.builder()
      .addConfiguration(MASConfiguration.builder()
        .addEventHandler(AddDepotEvent.class, AddDepotEvent.defaultHandler())
        .addEventHandler(AddParcelEvent.class, AddParcelEvent.defaultHandler())
        .addEventHandler(AddVehicleEvent.class, CustomVehicleHandler.INSTANCE)
        .addEventHandler(TimeOutEvent.class, TimeOutEvent.ignoreHandler())
        .build())
      .addScenario(createScenario())
      .repeat(1)
      .withRandomSeed(0)
      .withThreads(1)
      .usePostProcessor(new ExamplePostProcessor())
      .showGui(View.builder()
        .with(GraphRoadModelRenderer.builder().withMargin(2))
        .with(RoadUserRenderer.builder()
          .withImageAssociation(AgvAgent.class, "/graphics/flat/forklift2.png"))
        .with(PDPModelRenderer.builder())
        .with(CommRenderer.builder()
          .withReliabilityColors()
          .withMessageCount())
        .with(TimeLinePanel.builder())
        .withResolution((int) RESOLUTION.x, (int) RESOLUTION.y)
        .withAutoPlay()
        .withSpeedUp(uiSpeedUp)
        .withTitleAppendix("Experiments example"))
      .perform(System.out, arguments);

    // Handle experiment results
    if (results.isPresent()) {
      for (final SimulationResult sr : results.get().getResults()) {
        System.out.println(
          sr.getSimArgs().getRandomSeed() + " " + sr.getResultObject());
      }
    } else {
      throw new IllegalStateException("Experiment did not complete.");
    }
  }

  /**
   * Creates the scenario for the experiment.
   * Includes vehicles, parcels, depots, and road models.
   */
  @SuppressWarnings("unchecked")
  static Scenario createScenario() {
    // Generate random depot locations
    Point point_dep;
    for (int i = 0; i <= DEPOT_NUMBER - 1; i++) {
      double random_x = Math.random();
      double random_y = Math.random();
      final double xcoor = random_x * (MAX_WIDTH + 1) - (random_x * (MAX_WIDTH + 1)) % MIN_HOR;
      final double ycoor = random_y * (MAX_HEIGHT + 1) - (random_y * (MAX_HEIGHT + 1)) % MIN_VER;
      point_dep = new Point(xcoor, ycoor);
      point_dep_list.add(point_dep);
    }

    // Create lists of vehicles, parcels, and depots
    final Iterable<AddVehicleEvent> list_of_vehicles = create_VehicleEvents(VEHICLES_NUMBER);
    final Iterable<AddParcelEvent> list_of_parcels = create_ParcelEvents(PARCELS_NUMBER);
    final Iterable<AddDepotEvent> list_of_depots = create_DepotEvents(DEPOT_NUMBER);

    return Scenario.builder()
      .addEvents(list_of_vehicles)
      .addEvents(list_of_parcels)
      .addEvents(list_of_depots)
      .addEvent(TimeOutEvent.create(EXP_TIME_DURATION))
      .scenarioLength(EXP_TIME_DURATION)
      .addModel(RoadModelBuilders.dynamicGraph(new ListenableGraph<>(
        createGrid(WIDTH, HEIGHT, VERTICAL_SPACING, HORIZONTAL_SPACING, SPACING)))
        .withCollisionAvoidance()
        .withDistanceUnit(SI.METER)
        .withVehicleLength(VEHICLE_LENGTH))
      .addModel(CommModel.builder())
      .addModel(DefaultPDPModel.builder()
        .withTimeWindowPolicy(TimeWindowPolicies.LIBERAL))
      .build();
  }

  /**
   * Generates vehicle events for the scenario.
   */
  static Iterable<AddVehicleEvent> create_VehicleEvents(int nbVehicles) {
    // Implementation
  }

  /**
   * Generates parcel events for the scenario.
   */
  static Iterable<AddParcelEvent> create_ParcelEvents(int nbParcels) {
    // Implementation
  }

  /**
   * Generates depot events for the scenario.
   */
  static Iterable<AddDepotEvent> create_DepotEvents(int nbDepots) {
    // Implementation
  }

  /**
   * Creates a grid graph for the road model.
   */
  static Graph<LengthData> createGrid(int width, int height, int vertSpacing, int horSpacing, double spacing) {
    final Graph<LengthData> graph = new MultimapGraph<>();
    // Grid creation logic
    return graph;
  }
}
