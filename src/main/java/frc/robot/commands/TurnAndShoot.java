/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new TurnAndShoot.
   */
  public TurnAndShoot(DriveTrain robotDrive, Shooter shooter, Carousel carousel, CarouselCommand carouselCommand, DriveCommand driveCommand, boolean rescheduleDriveCommand) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(new FaceShootingTarget(robotDrive, 1.5, driveCommand, shooter),
               new ShooterCommand(shooter, carousel, robotDrive,/* 0,*/ carouselCommand,/* driveCommand,*/ rescheduleDriveCommand));
  }
}
