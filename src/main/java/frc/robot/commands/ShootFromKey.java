/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Shooter;

public class ShootFromKey extends CommandBase {
  /**
   * Creates a new ShootFromKey.
   */
  Shooter shooter;
  Carousel carousel;
  CarouselCommand carouselCommand;
 
  int initialCarouselTicks;

  public ShootFromKey(Shooter shooter, Carousel carousel, CarouselCommand carouselCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.carouselCommand = carouselCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 0.00001);
    carouselCommand.cancel();
    initialCarouselTicks = carousel.getTicks();
    shooter.setHood(150);
    shooter.setShooterRpm(4000.0);
    shooter.setTurretAdjusted(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getSpeed() > 3650) {
      carousel.spin(Constants.CAROUSEL_SHOOTER_SPEED);
      shooter.shoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    carousel.spin(0.0);
    carousel.resetBallCount();
    carouselCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (carousel.getTicks() - initialCarouselTicks) < -5 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
  }
}