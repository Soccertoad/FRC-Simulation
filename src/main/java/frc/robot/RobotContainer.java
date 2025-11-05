// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.SuperStructure.SuperStructure;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.generated.Telemetry;
import frc.robot.Subsystems.Swerve.generated.TunerConstants;
import frc.robot.Util.AIRobotInSimulation2024;

public class RobotContainer {
  private final CommandXboxController Driver = new CommandXboxController(0);
  private final CommandXboxController Operator = new CommandXboxController(1);
  private final CommandXboxController TEST = new CommandXboxController(5);

  private Elevator elevator;
  private SuperStructure superStructure;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {

    switch(Constants.CURRENT_MODE){
      case REAL ->{

      }

      case SIM -> {
        elevator = new Elevator(new ElevatorIOSim());

        drivetrain.resetPose(new Pose2d(3,3, new Rotation2d()));
        resetSimAuto();
        AIRobotInSimulation2024.startOpponentRobotSimulations();
      }

      case REPLAY -> {
        elevator = new Elevator(new ElevatorIO() {});
      }
    }
    
    superStructure = new SuperStructure(elevator);

    configureBindings();

    if(Constants.LIVE_TUNING){
      programmingTestBindings();
    }
  }

  private void configureBindings() {
    
    Operator.a().whileTrue(elevator.setPosition(Feet.of(5)));
    Operator.b().onTrue(elevator.setPosition(Feet.of(8.9)));
    Operator.x().onTrue(elevator.setPosition(Feet.of(1)));
    //elevator.setDefaultCommand(elevator.setPosition(Feet.of(elevatorHeight.get())));
    
    //elevator.setDefaultCommand(elevator.setPosition(funky::getLeftY));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-Driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    Driver.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    Driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void programmingTestBindings(){
    TEST.a().and(TEST.rightBumper()).whileTrue(elevator.runQStaticSysId(SysIdRoutine.Direction.kForward));
    TEST.b().and(TEST.rightBumper()).whileTrue(elevator.runQStaticSysId(SysIdRoutine.Direction.kReverse));
    TEST.x().and(TEST.rightBumper()).whileTrue(elevator.runDynamicSysId(SysIdRoutine.Direction.kForward));
    TEST.y().and(TEST.rightBumper()).whileTrue(elevator.runDynamicSysId(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void resetSimAuto(){
    SimulatedArena.getInstance().resetFieldForAuto();
  }
  public void simPeriodic(){
    SimulatedArena.getInstance().simulationPeriodic();
    
    Logger.recordOutput(
      "FieldSimulation/Coral",
      SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
      "FieldSimulation/Algae",
      SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
      "FieldSimulation/Alliance-Robots", 
      AIRobotInSimulation2024.getAlliancePartnerRobotPoses()
    );
    Logger.recordOutput(
      "FieldSimulation/Opposing-Robots", 
      AIRobotInSimulation2024.getOpponentRobotPoses()
    );
  }
}
