package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim physicsSim;
  private PIDController controller;
  private double updateVoltage = 0;
  private double desiredPosition;
  private boolean positionControl = false;

  public ElevatorIOSim(){

    physicsSim = new ElevatorSim(
      ElevatorConstants.motorSim, 
      ElevatorConstants.totalReduction, 
      ElevatorConstants.carrageMass.in(Kilograms), 
      ElevatorConstants.drumRadiusMeters, 
      0, 
      ElevatorConstants.maxExtension.in(Meters), 
      true, 
      0 
    );

    controller = new PIDController(7, 0, 0);
    physicsSim.update(0.02);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){

    if(positionControl){
      updateVoltage = controller.calculate(inputs.positionMeters, desiredPosition);
    }

    physicsSim.setInputVoltage(updateVoltage);

    physicsSim.update(0.02);

    inputs.positionMeters = physicsSim.getPositionMeters();
    inputs.appliedVolts = updateVoltage;
    inputs.appliedVolts = physicsSim.getCurrentDrawAmps();
    inputs.velocityMetersPerSecond = physicsSim.getVelocityMetersPerSecond();
    inputs.desiredPositionMeters = desiredPosition;
  }

  @Override
  public void setPosition(Distance distance){
    positionControl = true;
    desiredPosition = distance.in(Meter);
  }

  @Override
  public void setVoltage(Voltage voltage){
    positionControl = false;
    updateVoltage = voltage.in(Volt);
  }

  @Override
  public void setPID(double p, double i, double d, double feedforward){
    controller.setPID(p, i, d);
  }

  @Override
  public void stop(){
    positionControl = false;
  }
}
