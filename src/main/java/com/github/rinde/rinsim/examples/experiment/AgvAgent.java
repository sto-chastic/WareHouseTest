package com.github.rinde.rinsim.examples.experiment;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.Model.AbstractModel;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.Message;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.GraphRoadModel;
import com.github.rinde.rinsim.core.model.road.MovingRoadUser;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModels;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.collect.ImmutableList;

public class AgvAgent extends Vehicle implements MovingRoadUser, CommUser {

    // Constants for configuration
    private static final double MIN_RANGE = 0.2;
    private static final double MAX_RANGE = 1.5;
    private static final long LONELINESS_THRESHOLD = 10_000; // 10 seconds
    private static final double SPEED = 1000.0;

    // Communication and road-related attributes
    private final double range;
    private final double reliability;
    private Optional<RoadModel> roadModel = Optional.empty();
    private Optional<CommDevice> device = Optional.empty();

    // Destination points
    private Optional<Point> destination = Optional.empty();
    private Optional<Point> safePositionDestination = Optional.empty();
    private Optional<Point> normalOperationDestination = Optional.empty();

    // Current path and message information
    private Optional<List<Point>> pathToFollow = Optional.empty();
    private Optional<List<Point>> otherAgvRoute = Optional.empty();

    // Parcel handling
    private Optional<Parcel> currentParcel = Optional.empty();

    // State tracking
    private boolean collisionDanger = false;
    private long lastReceiveTime = 0;

    // AGV IDs tracking
    public static List<String> agvIds = new LinkedList<>();

    /**
     * Constructor to initialize the AGV with its VehicleDTO properties.
     * Configures the communication range and reliability.
     *
     * @param dto The vehicle data transfer object.
     */
    public AgvAgent(VehicleDTO dto) {
        super(dto);
        this.range = MIN_RANGE + Math.random() * (MAX_RANGE - MIN_RANGE);
        this.reliability = 0.9;
    }

    /**
     * Sets up the communication device for the AGV.
     * Configures the range and reliability.
     *
     * @param builder The communication device builder.
     */
    @Override
    public void setCommDevice(CommDeviceBuilder builder) {
        if (range >= 0) {
            builder.setMaxRange(range);
        }
        this.device = Optional.of(builder.setReliability(reliability).build());
    }

    /**
     * Main tick method called at every time step.
     * Handles parcel operations and communication.
     *
     * @param time The current time lapse.
     */
    @Override
    protected void tickImpl(TimeLapse time) {
        if (!time.hasTimeLeft()) return; // Stop if no time left

        RoadModel rm = getRoadModel();
        PDPModel pm = getPDPModel();

        handlePackage(rm, pm, time); // Manage package pickup/delivery
        handleMessages(time); // Handle communication with other AGVs
    }

    /**
     * Handles package-related tasks, such as pickup and delivery.
     *
     * @param rm   The road model.
     * @param pm   The PDP model.
     * @param time The current time lapse.
     */
    private void handlePackage(RoadModel rm, PDPModel pm, TimeLapse time) {
        if (!currentParcel.isPresent()) {
            // Find the closest parcel if none is assigned
            currentParcel = Optional.ofNullable(
                RoadModels.findClosestObject(rm.getPosition(this), rm, Parcel.class));
        }

        if (currentParcel.isPresent() && !collisionDanger) {
            Parcel parcel = currentParcel.get();
            boolean inCargo = pm.containerContains(this, parcel);

            // If the parcel is no longer valid, clear it
            if (!inCargo && !rm.containsObject(parcel)) {
                currentParcel = Optional.empty();
            } else if (inCargo) {
                // Parcel is in cargo, proceed to delivery location
                destination = Optional.of(parcel.getDeliveryLocation());
                if (rm.getPosition(this).equals(parcel.getDeliveryLocation())) {
                    pm.deliver(this, parcel, time); // Deliver the parcel
                }
            } else {
                // Parcel is not in cargo, proceed to pickup location
                destination = Optional.of(parcel.getPickupLocation());
                if (rm.getPosition(this).equals(parcel.getPickupLocation())) {
                    pm.pickup(this, parcel, time); // Pickup the parcel
                }
            }

            moveAlongPath(rm, time); // Move towards the destination
        } else if (collisionDanger) {
            handleCollision(rm, time); // Handle collision scenarios
        }
    }

    /**
     * Handles collision situations by moving the AGV to a safe position.
     *
     * @param rm   The road model.
     * @param time The current time lapse.
     */
    private void handleCollision(RoadModel rm, TimeLapse time) {
        moveAlongPath(rm, time); // Continue moving along the path
        if (rm.getPosition(this).equals(destination.orElse(null))) {
            destination = safePositionDestination; // Switch to safe position
            if (rm.getPosition(this).equals(safePositionDestination.orElse(null))) {
                collisionDanger = false; // Resume normal operation
                destination = normalOperationDestination;
            }
        }
    }

    /**
     * Handles communication with other AGVs.
     *
     * @param time The current time lapse.
     */
    private void handleMessages(TimeLapse time) {
        if (device.isPresent() && device.get().getUnreadCount() > 0) {
            // Process unread messages
            lastReceiveTime = time.getStartTime();
            ImmutableList<Message> messages = device.get().getUnreadMessages();

            for (Message message : messages) {
                Mssgs msgContent = (Mssgs) message.getContents();
                otherAgvRoute = msgContent.getSenderRoute();

                // Check for potential collisions with received routes
                if (otherAgvRoute.isPresent() && pathToFollow.isPresent() && !collisionDanger) {
                    RouteChecker checker = new RouteChecker(
                        this, pathToFollow.get(), otherAgvRoute.get(), getRoadModel());
                    collisionDanger = checker.getCollisionDanger();

                    if (collisionDanger) {
                        normalOperationDestination = destination; // Save normal operation state
                        destination = checker.getPreviousCrossRoad(); // Set new destination
                        safePositionDestination = checker.getBestPoint(); // Set safe point
                    }
                }
            }
        } else if (device.isPresent() && destination.isPresent()) {
            // Broadcast current route to other AGVs
            Mssgs newMsg = new Mssgs();
            newMsg.setSenderID(this.toString());
            newMsg.setSenderRoute(pathToFollow.orElse(null));
            newMsg.setEucDistance(calculateEuclideanDistance());
            device.get().broadcast(newMsg);
        }
    }

    /**
     * Moves the AGV along its planned path.
     *
     * @param rm   The road model.
     * @param time The current time lapse.
     */
    private void moveAlongPath(RoadModel rm, TimeLapse time) {
        if (destination.isPresent()) {
            pathToFollow = Optional.of(rm
