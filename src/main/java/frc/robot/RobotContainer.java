// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.SuperStructure.SuperStructure;

public class RobotContainer {
  private final CommandXboxController Operator = new CommandXboxController(1);
  
  @SuppressWarnings("unused")
  private SuperStructure superStructure;

  private Elevator elevator;
  
  public RobotContainer() {

    switch(Constants.CURRENT_MODE){
      case REAL ->{
        elevator = new Elevator(new ElevatorIOTalonFX());
      }

      case SIM -> {
        elevator = new Elevator(new ElevatorIOSim());
      }

      case REPLAY -> {
        elevator = new Elevator(new ElevatorIO() {});
      }
    }
    configureBindings();

    superStructure = new SuperStructure(elevator);
  }

  private void configureBindings() {
    
    Operator.a().whileTrue(elevator.setPosition(Feet.of(5)));
    Operator.b().onTrue(elevator.setPosition(Feet.of(8.9)));
    Operator.x().onTrue(elevator.setPosition(Feet.of(1)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
