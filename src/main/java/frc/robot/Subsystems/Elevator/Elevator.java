// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.TunableNumber;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TunableNumber testing = new TunableNumber("Elevator/Test", 5);

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO m_io) {
    this.io = m_io; 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    SmartDashboard.putNumber("Current Value", testing.getAsDouble());

  }

  public Command setPosition(Distance distance){
    return runOnce(()->{
      io.setPosition(distance);});
  }

  public double getPositionMeters(){
    return inputs.positionMeters;
  }

  public double getDesiredPosition(){
    return inputs.desiredPositionMeters;
  }
}
