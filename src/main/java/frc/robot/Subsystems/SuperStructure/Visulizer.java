package frc.robot.Subsystems.SuperStructure;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private Distance firstStageEnd = Feet.of(3);
  private Distance secondStageEnd = Feet.of(6);
  // stage thickness times 2 for being rectangle
  private Distance stageThickness = Inches.of(1).times(2);
  
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

  public void update(double elevatorHeightMeters, boolean isTilted){
    if(!isTilted){
      elevator.setAngle(uprightElevator.in(Degrees));
      elevatorTilt = new Rotation3d(0, uprightElevator.in(Radian), 0);
    }
    elevator.setLength(elevatorHeightMeters);
    Logger.recordOutput("Mech2d/" + name, mech);


    // double stage2Height = -Math.max(
    //   // Math.max(
    //   //   heightFromBottom - thirdStage.in(Meters), 
    //   //   heightFromBottom - secondStage.in(Meters)
    //   // ), 
    //   heightFromBottom - thirdStage.in(Meters),
    //   0.0
    // );

    double heightFromBottom = elevatorHeightMeters + (stageThickness.in(Meters) * 3);
    /* 
    double stage2Height = -Math.max(
      0.0,
      Math.min(
        heightFromBottom - firstStageEnd.in(Meter),
        secondStageEnd.in(Meter)
      )
    );
    double stage3Height = -Math.max(
      0.0, 
      heightFromBottom - secondStageEnd.in(Meter)
    );
    */
    double stage2Height = -heightFromBottom/3;
    double stage3Height = -heightFromBottom/3;
    
    Translation3d stage1 = new Translation3d(elevatorOrigin.getY(), elevatorTilt);

    Translation3d stage2 = stage1.plus(new Translation3d(stage2Height, elevatorTilt));

    Translation3d stage3 = stage2.plus(new Translation3d(stage3Height, elevatorTilt));

    Translation3d carrageHeight = stage3.plus(new Translation3d(-heightFromBottom/3, elevatorTilt));

    Translation3d manipulatorPosition = carrageHeight;

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
          carrageHeight
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

  public void update(Distance elevatorHeightMeters, boolean isTilted){
    update(elevatorHeightMeters.in(Meters), isTilted);
  }

}
