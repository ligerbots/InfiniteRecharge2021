
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionMode;

public class ShooterCommand extends CommandBase {
  /*
   * Creates a new ShooterCommand.
   */

  Shooter shooter;
  Carousel carousel;

  ShooterPIDTuner pidTuner;
  CarouselCommand carouselCommand;

  double startTime;
  double shooterTargetSpeed;
  double initialCarouselSlot;
  double stableRPMTime;
 
  public enum ControlMethod {
    ACQUIRING, // Acquiring vision target
    SPIN_UP, // PIDF to desired RPM
    WAITING_FOR_STABILITY, //Wait two seconds to shoot
    HOLD, // Keep the shooter wheel at specific RPM, check to see if all systems ready
    FIRING, //Fire the shooter
  }

  ControlMethod currentControlMode;

  public ShooterCommand(Shooter shooter, Carousel carousel,  CarouselCommand carouselCommand) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.carousel = carousel;
    // need to control carousel manually
    // addRequirements(carousel);
    this.carouselCommand = carouselCommand;
    pidTuner = new ShooterPIDTuner(shooter);
  }

  private void rapidFire() {
    shooter.shoot();
    carousel.spin(0.8);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    carouselCommand.cancel();
    SmartDashboard.putString("shooter/Shooting", "Shoot");
    shooterTargetSpeed = 0.0;
    startTime = Robot.time();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    currentControlMode = ControlMethod.ACQUIRING;
    //starts spinning up the shooter to hard-coded PID values
    pidTuner.spinUpTune();
    // store current carouselTick value
    initialCarouselSlot = carousel.getSlot();

    // Do we need this?
    // shooter.setTurretAdjusted(0.0/*-Robot.angleErrorAfterTurn*/);
  }

  @Override
  public void execute() {
    SmartDashboard.putString("shooter/Shooting", currentControlMode.toString());

    if (currentControlMode == ControlMethod.ACQUIRING) {
      // Waiting for vision to acquire target and preparing turret
      if (shooter.vision.getStatus()) {
        double distance = shooter.vision.getDistance();
        double angleError = shooter.vision.getRobotAngle();
        shooter.setTurretAdjusted(angleError);
        shooterTargetSpeed = shooter.prepareShooter(distance);
        currentControlMode = ControlMethod.SPIN_UP;
      }
    }

    if (currentControlMode == ControlMethod.SPIN_UP) {
      // Shooter wheel warms up
      if (shooter.speedOnTarget(shooterTargetSpeed, 15)) {
        stableRPMTime = Robot.time();
        currentControlMode = ControlMethod.WAITING_FOR_STABILITY;
      }
    }

    if (currentControlMode == ControlMethod.WAITING_FOR_STABILITY) {
      // Waits for shooter wheel to be in range for 0.2 seconds 
      if (shooter.speedOnTarget(shooterTargetSpeed, 15)) {
        if (Robot.time() - stableRPMTime > 0.2) {
          //locks shooter speed in place
          pidTuner.HoldTune();
          currentControlMode = ControlMethod.HOLD;
        }
      } else {
        currentControlMode = ControlMethod.SPIN_UP;
      }
    }

    if (currentControlMode == ControlMethod.HOLD) {
      // Checks to see if the robot is ready to fire
      boolean speedOnTarget = shooter.speedOnTarget(shooterTargetSpeed, 8) || (Robot.time() - startTime > 3.5);
      boolean hoodOnTarget = Robot.time() - startTime > 0.75;
      if (speedOnTarget && hoodOnTarget) {
        rapidFire();
        currentControlMode = ControlMethod.FIRING;
      }
    }
    // Nothing to do in ControlMethod.FIRING
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.vision.setMode(VisionMode.INTAKE);
    carousel.resetBallCount();
    carousel.spin(0.0);
    carouselCommand.schedule();
    SmartDashboard.putString("shooter/Shooting", "Idle");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (carousel.getSlot() - initialCarouselSlot) > Constants.CAROUSEL_MAX_BALLS ||
      (currentControlMode == ControlMethod.ACQUIRING && Robot.time() - startTime > 2.0);
  }
}
