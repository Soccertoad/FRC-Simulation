// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SuperStructure;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator;

public class SuperStructure extends SubsystemBase {

  private Elevator elevator;

  private Visulizer realRobot = new Visulizer("Real Robot", Color.kDarkCyan);
  private Visulizer desiredRobot = new Visulizer("Desired Robot", Color.kOrange);

  
  /** Creates a new SuperStructure. */
  public SuperStructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    realRobot.update(elevator.getPositionMeters());
    desiredRobot.update(elevator.getDesiredPosition());
  }
}
