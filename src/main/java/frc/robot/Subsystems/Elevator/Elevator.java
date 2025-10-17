// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Util.TunableNumber;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  SysIdRoutine sysID = new SysIdRoutine(
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

  LoggedMechanism2d mech = new LoggedMechanism2d(Inches.of(32).in(Meters), ElevatorConstants.maxExtension.in(Meters));
  LoggedMechanismRoot2d root = mech.getRoot("BaseElevator", 2, 0);
  LoggedMechanismLigament2d elevator = root.append(new LoggedMechanismLigament2d("Elevator", getPositionMeters(), 90));

  TunableNumber elevatorKp = new TunableNumber("elevator Kp", 5);
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO m_io) {

    this.io = m_io; 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Mech2d", mech);
    elevator.setLength(getPositionMeters());

    if(elevatorKp.hasChanged()){
      io.setPID(elevatorKp.get(), 0, 0, 0);

    }
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
}
