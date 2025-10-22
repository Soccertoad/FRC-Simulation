// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

//import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private boolean isTilted = true;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO m_io) {

    this.io = m_io; 

  }

  private SysIdRoutine sysID = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, 
      null, 
      null, 
      (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())
    ), 
    new SysIdRoutine.Mechanism(
    (voltage) -> io.setVoltage(voltage), 
    null, 
    this)
  );

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

  }

  public Command setPosition(Distance distance){
    return runOnce(()->{
      io.setPosition(distance);});
  }

  public Command runQStaticSysId(SysIdRoutine.Direction direction){
    return sysID.quasistatic(direction);
  }

  public Command runDynamicSysId(SysIdRoutine.Direction direction){
    return sysID.dynamic(direction);
  }

  public double getPositionMeters(){
    return inputs.positionMeters;
  }

  public boolean isTilted(){
    if(getDesiredPosition() != 0.02){
      isTilted = false;
    }
    return isTilted;
  }

  public double getDesiredPosition(){
    return inputs.desiredPositionMeters;
  }
}
