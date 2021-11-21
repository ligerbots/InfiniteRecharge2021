/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMap;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndDriveAuto extends SequentialCommandGroup implements AutoCommandInterface {
  /**
   * Creates a new ShootAndDriveAuto.
   */
  public ShootAndDriveAuto(DriveTrain robotDrive, Shooter shooter, Intake intake, Carousel carousel, DriveCommand driveCommand, CarouselCommand carouselCommand) {

    TurnAndShoot shoot1 = new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, false);

    addCommands(shoot1, new MoveForward(robotDrive, driveCommand));
  }

  public Pose2d getInitialPose() {
    return FieldMap.startPosition[0];
  }
}
