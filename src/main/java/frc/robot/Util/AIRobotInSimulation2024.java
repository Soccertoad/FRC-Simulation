package frc.robot.Util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.Arrays;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

/**
 *
 *
 * <h2>Represents an AI robot during simulation.</h2>
 *
 * <p>This class models an AI-controlled robot used during simulation for driver practice and skill development.
 *
 * <p>The AI robots are capable of performing various tasks to enhance training scenarios, including:
 *
 * <ul>
 *   <li>Automatically cycling around the field to help practice offensive skills.
 *   <li>Delivering feed-shots to assist in practicing front-field clean-ups.
 *   <li>Driving via joystick control to simulate defense and counter-defense scenarios.
 * </ul>
 */
public class AIRobotInSimulation2024 extends SubsystemBase {
    /* If an opponent robot is not requested to be on the field, it is placed ("queens") outside the field at predefined positions. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };
    /* The robots will be teleported to these positions when teleop begins. */
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };
    /* Store instances of AI robots in a static array. */
    public static final AIRobotInSimulation2024[] instances = new AIRobotInSimulation2024[5];

    /* The drivetrain configuration for the opponent robots in the maple-sim simulation. */
    private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG =
            DriveTrainSimulationConfig.Default().withRobotMass(Pound.of(115));
    static CommandXboxController quickController = new CommandXboxController(5);
        /**
         *
         *
         * <h2>Activates the opponent robots.</h2>
         *
         * <p>The opponent robots <strong>will not</strong> appear on the field immediately. They are initially placed at
         * the {@link #ROBOT_QUEENING_POSITIONS}.
         *
         * <p>Instead of being active right away, a sendable chooser is sent to the dashboard, allowing the user to select
         * the mode of these robots.
         */
        public static void startOpponentRobotSimulations() {
            try {
                // Creates an instance of the first AI robot
                instances[0] = new AIRobotInSimulation2024(0);
                // Builds the behavior chooser for the first AI robot
                instances[0].buildBehaviorChooser(
                        new CommandXboxController(3));
    
                // Same of the following:
    
                instances[1] = new AIRobotInSimulation2024(1);
                instances[1].buildBehaviorChooser(
                        new CommandXboxController(4));
    
                instances[2] = new AIRobotInSimulation2024(2);
                instances[2].buildBehaviorChooser(
                        quickController);

            instances[3] = new AIRobotInSimulation2024(3);
            instances[3].buildBehaviorChooser(
                    quickController);

            instances[4] = new AIRobotInSimulation2024(4);
            instances[4].buildBehaviorChooser(
                    quickController);
        } catch (Exception e) {
            DriverStation.reportError("failed to load opponent robot simulation path, error:" + e.getMessage(), false);
        }
    }

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    public AIRobotInSimulation2024(int id) {
        this.id = id;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation =
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(DRIVETRAIN_CONFIG, queeningPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    /**
     * Builds the behavior chooser for this opponent robot and sends it to the dashboard. This allows the user to select
     * the robot's behavior mode during simulation.
     */
    public void buildBehaviorChooser(
            CommandXboxController joystick) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();

        // Supplier to disable the robot: sets the robot's pose to the queening position and stops its movement
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                        .ignoringDisable(true);

        // The option to disable the robot
        behaviorChooser.setDefaultOption("Disable", joystickDrive(joystick));

        // The option for manual joystick control of the robot
        behaviorChooser.addOption("Joystick Drive", disable.get());

        // Schedule the selected command when another behavior is chosen
        behaviorChooser.onChange((Command::schedule));

        // Schedule the command when teleop mode is enabled
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(disable.get());

        // Display the behavior chooser on the dashboard for the user to select the desired robot behavior
        SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
    }

    /**
     * Joystick drive command for controlling the opponent robots. This command allows the robot to be driven using an
     * Xbox controller.
     */
    private Command joystickDrive(CommandXboxController joystick) {
        // Obtain chassis speeds from the joystick inputs
        final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Forward/Backward
                -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Left/Right
                -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond) // Rotation
                );

        // Obtain the driver station facing for the opponent alliance
        // Used in defense practice, where two tabs of AScope show the driver stations of both alliances
        final Supplier<Rotation2d> opponentDriverStationFacing = () ->
                FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

        return Commands.run(
                        () -> {
                            // Calculate field-centric speed from the driver station-centric speed
                            final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                    joystickSpeeds.get(),
                                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                            .plus(Rotation2d.fromDegrees(180)));
                            // Run the field-centric speed to control the robot's movement
                            driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                        },
                        this)
                // Before the command starts, reset the robot to its starting position on the field
                .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id])));
    }

    public static Pose2d[] getOpponentRobotPoses() {
        return getRobotPoses(new AIRobotInSimulation2024[] {instances[0], instances[1], instances[2]});
    }

    public static Pose2d[] getAlliancePartnerRobotPoses() {
        return getRobotPoses(new AIRobotInSimulation2024[] {instances[3], instances[4]});
    }

    private static Pose2d[] getRobotPoses(AIRobotInSimulation2024[] instances) {
        return Arrays.stream(instances)
                .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
                .toArray(Pose2d[]::new);
    }

   
}