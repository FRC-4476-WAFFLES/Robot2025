package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

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

    /* Motion Magic Gains */
    private final DoubleEntry MotionMagicCruiseVelocity; 
    private final DoubleEntry MotionMagicAcceleration; 
    private final DoubleEntry MotionMagicJerk; // If set to zero, there is no jerk limit
    private final DoubleEntry MotionMagicExpo_kA;
    private final DoubleEntry MotionMagicExpo_kV;

    private static ArrayList<NetworkConfiguredPID> onEnableUpdateList = new ArrayList<NetworkConfiguredPID>();
    private Runnable updateMethod;

    public NetworkConfiguredPID(String name) {
        this(name, () -> {});
    }

    public NetworkConfiguredPID(String name, Runnable updateMethod) {
        this.name = name;
        this.updateMethod = updateMethod;

        groupTable = inst.getTable("PID Configuration");
        instanceTable = groupTable.getSubTable(this.name);

        PTopic = instanceTable.getDoubleTopic("P Value").getEntry(0, (PubSubOption)null);
        ITopic = instanceTable.getDoubleTopic("I Value").getEntry(0, (PubSubOption)null);
        DTopic = instanceTable.getDoubleTopic("D Value").getEntry(0, (PubSubOption)null);

        GTopic = instanceTable.getDoubleTopic("Gravity Feedforward").getEntry(0, (PubSubOption)null);
        STopic = instanceTable.getDoubleTopic("Friction Feedforward").getEntry(0, (PubSubOption)null);
        VTopic = instanceTable.getDoubleTopic("Velocity Feedforward").getEntry(0, (PubSubOption)null);
        ATopic = instanceTable.getDoubleTopic("Acceleration Feedforward").getEntry(0, (PubSubOption)null);

        MotionMagicCruiseVelocity = instanceTable.getDoubleTopic("MotionMagic CruiseVelocity").getEntry(0, (PubSubOption)null);
        MotionMagicAcceleration = instanceTable.getDoubleTopic("MotionMagic Acceleration").getEntry(0, (PubSubOption)null);
        MotionMagicJerk = instanceTable.getDoubleTopic("MotionMagic Jerk").getEntry(0, (PubSubOption)null);
        MotionMagicExpo_kA = instanceTable.getDoubleTopic("MotionMagic kA").getEntry(0, (PubSubOption)null);
        MotionMagicExpo_kV = instanceTable.getDoubleTopic("MotionMagic kV").getEntry(0, (PubSubOption)null);


        onEnableUpdateList.add(this);
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

    /**
     * Gets the current MotionMagicCruiseVelocity value from networktables
     * @return the current value
     */
    public double getMotionMagicCruiseVelocity() {
        return MotionMagicCruiseVelocity.getAsDouble();
    }

    /**
     * Gets the current MotionMagicAcceleration value from networktables
     * @return the current value
     */
    public double getMotionMagicAcceleration() {
        return MotionMagicAcceleration.getAsDouble();
    }

    /**
     * Gets the current MotionMagicJerk value from networktables
     * @return the current value
     */
    public double getMotionMagicJerk() {
        return MotionMagicJerk.getAsDouble();
    }

    /**
     * Gets the current MotionMagicExpo_kA value from networktables
     * @return the current value
     */
    public double getMotionMagicExpo_kA() {
        return MotionMagicExpo_kA.getAsDouble();
    }

    /**
     * Gets the current MotionMagicExpo_kV value from networktables
     * @return the current value
     */
    public double getMotionMagicExpo_kV() {
        return MotionMagicExpo_kV.getAsDouble();
    }

    /* OnEnable Hook Setup */
    public void runUpdateTask() {
        updateMethod.run();
    }

    public static void onEnable() {
        for (var obj : onEnableUpdateList) {
            obj.runUpdateTask();
        }
    }
}
