// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Util.TunableNumber;

public class RobotContainer {
  private final CommandXboxController Driver = new CommandXboxController(0);
  private final CommandXboxController Operator = new CommandXboxController(1);
  private final CommandXboxController TEST = new CommandXboxController(5);

  Elevator elevator;
  public RobotContainer() {

    switch(Constants.CURRENT_MODE){
      case REAL ->{

      }

      case SIM -> {
        elevator = new Elevator(new ElevatorIOSim());
      }

      case REPLAY -> {
        elevator = new Elevator(new ElevatorIO() {});
      }
    }

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
}
