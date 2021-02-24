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

public class TurnShootTurnBack extends SequentialCommandGroup {
    /**
     * Creates a new TurnAndShoot.
     */
    DriveCommand m_driveCommand;

    public TurnShootTurnBack(DriveTrain driveTrain, Shooter shooter, Carousel carousel, 
            CarouselCommand carouselCommand, DriveCommand driveCommand, double targetHeading) 
    {
        // We don't want the drive command turned on and off in the middle, so handle it here
        m_driveCommand = driveCommand;

        // Turn to face the target with accuracy 3 degrees. The offset will be handled by the turret.
        // This is faster than trying to be too accurate.
        // For turning back, keep the accuracy tight (1 degree).
        addCommands(new FaceShootingTarget(driveTrain, 3.0, null, shooter),
                    new ShooterCommand(shooter, carousel, driveTrain, carouselCommand, false),
                    new TurnToHeading(driveTrain, targetHeading, 1.0));
    }

    @Override
    public void initialize() {
        if (m_driveCommand != null) m_driveCommand.cancel();

        // SeqCommandGroup has an initialize() method, which HAS to be called.
        // This initializes the first command (FaceShooter), so do this after the local init stuff.
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted)
            System.out.println("*** TurnShootTurnBack interrupted ***");

        if (m_driveCommand != null) m_driveCommand.schedule();
        System.out.println("TurnShootTurnBack ended. interrupted = " + interrupted );
    }
}
