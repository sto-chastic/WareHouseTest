package com.github.rinde.rinsim.examples.experiment;

import java.util.List;
import com.github.rinde.rinsim.core.model.comm.MessageContents;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;

/**
 * The Mssgs class implements the MessageContents interface from RinSim
 * and is used to define the structure and data for communication messages 
 * exchanged between agents in the simulation.
 */
public class Mssgs implements MessageContents {

    // Fields for various pieces of information that may be included in a message
    Optional<String> sendingMessage;              // Message content or handshake information
    Optional<List<List<Point>>> routingInfo;      // Detailed routing information (list of routes)
    Optional<List<Point>> senderRoute;            // Route of the sender agent
    Optional<Long> timeStamp;                     // Timestamp of when the message was sent
    Optional<List<String>> aGV_IDs;               // List of AGV (Automated Guided Vehicle) IDs
    Optional<String> senderID;                    // ID of the sender agent
    Optional<Double> eucDistanceToTarget;         // Euclidean distance to the target

    /**
     * Default constructor initializes all fields as absent using Guava's Optional.
     * This ensures safe handling of null values.
     */
    public Mssgs() {
        sendingMessage = Optional.absent();
        routingInfo = Optional.absent();
        senderRoute = Optional.absent();
        timeStamp = Optional.absent();
        aGV_IDs = Optional.absent();
        senderID = Optional.absent();
        eucDistanceToTarget = Optional.absent();
    }

    // --- Setter Methods ---

    /**
     * Sets the timestamp of the message.
     *
     * @param timeS The timestamp in milliseconds.
     */
    public void setTimeStamp(long timeS) {
        timeStamp = Optional.of(timeS);
    }

    /**
     * Sets the Euclidean distance to the target.
     *
     * @param dist The distance as a double value.
     */
    public void setEucDistance(double dist) {
        eucDistanceToTarget = Optional.of(dist);
    }

    /**
     * Sets the handshake or message content.
     *
     * @param Msg The content of the message.
     */
    public void handshake(String Msg) {
        sendingMessage = Optional.of(Msg);
    }

    /**
     * Sets the route of the sender agent.
     *
     * @param Route A list of Points representing the sender's route.
     */
    public void setSenderRoute(List<Point> Route) {
        senderRoute = Optional.of(Route);
    }

    /**
     * Sets the ID of the sender agent.
     *
     * @param ID The sender's ID as a string.
     */
    public void setSenderID(String ID) {
        senderID = Optional.of(ID);
    }

    // --- Getter Methods ---

    /**
     * Gets the sender's route.
     *
     * @return An Optional containing the sender's route if present.
     */
    public Optional<List<Point>> getSenderRoute() {
        return senderRoute;
    }

    /**
     * Gets the sender's ID.
     *
     * @return The sender's ID as a string.
     */
    public String getSenderID() {
        return senderID.get();  // Assumes senderID is always present
    }

    /**
     * Gets the timestamp of the message.
     *
     * @return The timestamp as a long value.
     */
    public long getTimeStamp() {
        return timeStamp.get();  // Assumes timestamp is always present
    }

    /**
     * Gets the Euclidean distance to the target.
     *
     * @return The distance as a double value.
     */
    public double getEucDistance() {
        return eucDistanceToTarget.get();  // Assumes distance is always present
    }

    /**
     * Gets the handshake or message content.
     *
     * @return An Optional containing the message content if present.
     */
    public Optional<String> getHandshake() {
        return sendingMessage;
    }
}
