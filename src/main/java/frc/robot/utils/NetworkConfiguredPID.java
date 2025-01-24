package frc.robot.utils;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;

/**
 * A utility that provides a quick and easy way to configure a PID controller from networktables
 * It is reccomended to only update gains on the PID controller when enabling the robot, to avoid erratic behaviour while editing 
 */
public class NetworkConfiguredPID {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final String name;

    private final NetworkTable groupTable;
    private final NetworkTable instanceTable;

    /* Standard Gains */
    private final DoubleEntry PTopic;
    private final DoubleEntry ITopic;
    private final DoubleEntry DTopic;

    /* Specialized Gains */
    private final DoubleEntry GTopic; // Output to overcome gravity (output)
    private final DoubleEntry STopic; // Output to overcome static friction (output)
    private final DoubleEntry VTopic; // Output per unit of requested velocity (output/rps)
    private final DoubleEntry ATopic; // Output per unit of requested acceleration (output/(rps/s))


    public NetworkConfiguredPID(String name) {
        this.name = name;

        groupTable = inst.getTable("PID Configuration");
        instanceTable = groupTable.getSubTable(this.name);

        PTopic = instanceTable.getDoubleTopic("P Value").getEntry(0, (PubSubOption)null);
        ITopic = instanceTable.getDoubleTopic("I Value").getEntry(0, (PubSubOption)null);
        DTopic = instanceTable.getDoubleTopic("D Value").getEntry(0, (PubSubOption)null);

        GTopic = instanceTable.getDoubleTopic("Gravity Feedforward").getEntry(0, (PubSubOption)null);
        STopic = instanceTable.getDoubleTopic("Friction Feedforward").getEntry(0, (PubSubOption)null);
        VTopic = instanceTable.getDoubleTopic("Velocity Feedforward").getEntry(0, (PubSubOption)null);
        ATopic = instanceTable.getDoubleTopic("Acceleration Feedforward").getEntry(0, (PubSubOption)null);
    }

    /**
     * Gets the current P value from networktables
     * @return the current value
     */
    public double getP() {
        return PTopic.getAsDouble();
    }

    /**
     * Gets the current I value from networktables
     * @return the current value
     */
    public double getI() {
        return ITopic.getAsDouble();
    }

    /**
     * Gets the current D value from networktables
     * @return the current value
     */
    public double getD() {
        return DTopic.getAsDouble();
    }

    /**
     * Gets the current G value from networktables
     * @return the current value
     */
    public double getG() {
        return GTopic.getAsDouble();
    }

    /**
     * Gets the current S value from networktables
     * @return the current value
     */
    public double getS() {
        return STopic.getAsDouble();
    }

    /**
     * Gets the current V value from networktables
     * @return the current value
     */
    public double getV() {
        return VTopic.getAsDouble();
    }

    /**
     * Gets the current A value from networktables
     * @return the current value
     */
    public double getA() {
        return ATopic.getAsDouble();
    }
}
