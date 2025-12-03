package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO{

  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private PositionVoltage positionPID = new PositionVoltage(0).withSlot(0);
  private double desiredPosition;

  public ElevatorIOTalonFX(){
    elevatorMotor.setPosition(0);
    elevatorMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.tempCelcius = elevatorMotor.getDeviceTemp().refresh().getValueAsDouble();
    inputs.positionMeters = elevatorMotor.getPosition().refresh().getValueAsDouble();
    inputs.velocityMetersPerSecond = elevatorMotor.getVelocity().refresh().getValueAsDouble();
    inputs.desiredPositionMeters = desiredPosition;

    inputs.appliedVolts = elevatorMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.statorCurrent = elevatorMotor.getStatorCurrent().refresh().getValueAsDouble();
    inputs.supplyCurrent = elevatorMotor.getSupplyCurrent().refresh().getValueAsDouble();
  }

  @Override
  public void setPosition(Distance position){
    desiredPosition = position.in(Meter);
    elevatorMotor.setControl(positionPID.withPosition(position.in(Meters)));
  }

  @Override
  public void setVoltage(Voltage voltage){
    elevatorMotor.setVoltage(voltage.in(Volt));
  }

  @Override
  public void setPID(double p, double i, double d, double feedforward){
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    elevatorMotor.getConfigurator().apply(config);
  }

  @Override
  public void stop(){
    elevatorMotor.set(0);
  }
}