/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class TurnShootTurnBack extends SequentialCommandGroup {
    /**
     * Creates a new TurnAndShoot.
     */
    Command m_driveCommand;

    public TurnShootTurnBack(DriveTrain driveTrain, Shooter shooter, Carousel carousel, 
            CarouselCommand carouselCommand, DriveCommand driveCommand) 
    {
        // We don't want the drive command turned on and off in the middle, so handle it here
        m_driveCommand = driveCommand;
        
        double originalHeading = driveTrain.getHeading();
        addCommands(new FaceShootingTarget(driveTrain, 1.5, null, shooter),
                    new ShooterCommand(shooter, carousel, driveTrain, carouselCommand, false),
                    new TurnToHeading(driveTrain, null, originalHeading, 1.5));
    }

    @Override
    public void initialize() {
        if (m_driveCommand != null) m_driveCommand.cancel();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_driveCommand != null) m_driveCommand.schedule();
    }
}
