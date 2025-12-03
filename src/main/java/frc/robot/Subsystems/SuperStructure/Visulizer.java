package frc.robot.Subsystems.SuperStructure;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator.ElevatorConstants;

public class Visulizer {

  private String name;
  private LoggedMechanism2d mech = new LoggedMechanism2d(
    Constants.ROBOT_WIDTH.in(Meters), 
    ElevatorConstants.maxExtension.in(Meters), 
    new Color8Bit(Color.kBlack)
  );
  private LoggedMechanismRoot2d elevatorRoot;
  private LoggedMechanismLigament2d elevator;

  private Angle uprightElevator = Degree.of(90);
  private Angle startingTilt = Degree.of(-45);
  Rotation3d elevatorTilt = new Rotation3d(0, startingTilt.in(Radian), 0);
  Translation3d elevatorOrigin = new Translation3d(0.37, 0, 0.08);

  private Distance firstStageEnd = Inch.of(35.2);
  private Distance secondStageEnd = Inch.of(35.825);
  private Distance carrageLength = Inch.of(6);

  // stage thickness 
  private Distance stageThickness = Inch.of(1);
  
  public Visulizer(String name, Color color){
    this.name = name;
    elevatorRoot = mech.getRoot(
      name + " Elevator Root", 
      elevatorOrigin.getX(),
      elevatorOrigin.getY()
    );

    elevator = elevatorRoot.append(
      new LoggedMechanismLigament2d(
        name + " Elevator", 
        ElevatorConstants.maxExtension.in(Meters), 
        ElevatorConstants.startingAngle.in(Degrees), 
        15,
        new Color8Bit(color)
      )
    );

    elevator.setLength(0.2);
  }

  public void update(double elevatorHeightMeters){
    if(true){
      elevatorTilt = new Rotation3d(0, uprightElevator.in(Radian), 0);
      elevator.setAngle(new Rotation2d(Degree.of(90)));
      
    }
    elevator.setLength(elevatorHeightMeters);
    Logger.recordOutput("Mech2d/" + name, mech);

    double heightFromBottom = elevatorHeightMeters + stageThickness.times(8).in(Meters) + elevatorOrigin.getY();

    double stage2Height = -Math.min(firstStageEnd.minus(Inch.of(4)).in(Meter), heightFromBottom/3);
    double stage3Height = -Math.min(secondStageEnd.minus(Inch.of(4)).in(Meter), heightFromBottom/3);
    double carrageHeight = -Math.min(secondStageEnd.minus(carrageLength).minus(stageThickness).in(Meter), heightFromBottom/3);

    Translation3d stage1 = new Translation3d(elevatorOrigin.getY(), elevatorTilt);

    Translation3d stage2 = stage1.plus(new Translation3d(stage2Height, new Rotation3d(0, /*elevatorTilt.getY()*/ Degree.of(90).in(Radian), 0)));

    Translation3d stage3 = stage2.plus(new Translation3d(stage3Height, new Rotation3d(0, /*elevatorTilt.getY()*/ Degree.of(90).in(Radian), 0)));

    Translation3d carrage = stage3.plus(new Translation3d(carrageHeight, new Rotation3d(0, /*elevatorTilt.getY()*/ Degree.of(90).in(Radian), 0)));

    Translation3d manipulatorPosition = carrage;

    Pose3d[] elevatorStages = {
      new Pose3d(
        elevatorOrigin.plus(
          stage1
        ), 
        Rotation3d.kZero
      ),
      new Pose3d(
        elevatorOrigin.plus(
          stage2
        ),
        Rotation3d.kZero
      ),
      new Pose3d(
        elevatorOrigin.plus(
          stage3
        ),
        Rotation3d.kZero
      ),
      new Pose3d(
        elevatorOrigin.plus(
          carrage
        ),
        Rotation3d.kZero
      ),
      new Pose3d(
        elevatorOrigin.plus(
          manipulatorPosition
        ),
        Rotation3d.kZero
      )
    };
    Logger.recordOutput("Robot3d/" + name, elevatorStages);
  }

  public void update(Distance elevatorHeightMeters){
    update(elevatorHeightMeters.in(Meters));
  }

}
