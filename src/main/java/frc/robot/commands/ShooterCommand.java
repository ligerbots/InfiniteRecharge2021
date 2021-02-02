
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionMode;

public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  double waitTime;
  double startTime;

  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;
  ShooterPIDTuner pidTuner;
  double shooterTargetSpeed;

  boolean startShooting;

  //CarouselCommand carouselCommand;
  //DriveCommand driveCommand;

  int initialCarouselTicks;

  long stableRPMTime;
  boolean startedTimerFlag;
  boolean foundTarget;
  boolean setPid;
 
  public enum ControlMethod {
    SPIN_UP, // PIDF to desired RPM
    HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
  }

  ControlMethod currentControlMode;
  boolean rescheduleDriveCommand;

  public ShooterCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive, double waitTime, /*CarouselCommand carouselCommand, DriveCommand driveCommand,*/ boolean rescheduleDriveCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;
    this.waitTime = waitTime;
    //this.carouselCommand = carouselCommand;
    //this.driveCommand = driveCommand;
    this.rescheduleDriveCommand = rescheduleDriveCommand;
    startShooting = false;
    pidTuner = new ShooterPIDTuner(shooter);
  }

  public void rapidFire() {
    carousel.spin(0.8);
    shooter.shoot();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    foundTarget = false;
    shooterTargetSpeed = 0.0;
    // This flag is used so we only set the PID values once per command. We don't want to constantly reset the PID
    // values  in the execute() method.
    setPid = true;

    // driveCommand.cancel();
    startTime = System.nanoTime();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    // carouselCommand.cancel();
    currentControlMode = ControlMethod.SPIN_UP;
    //starts spinning up the shooter to hard-coded PID values
    pidTuner.spinUpTune();
    System.out.println("Initial NavX Heading: " + robotDrive.getHeading());
    // store current carouselTick value
    initialCarouselTicks = carousel.getTicks();

    angleError = shooter.vision.getRobotAngle();
    distance = shooter.vision.getDistance();

    currentControlMode = ControlMethod.SPIN_UP;
    startedTimerFlag = false;
    System.out.println("Initial Angle Offset: " + angleError);
    shooter.setTurretAdjusted(0/*-Robot.angleErrorAfterTurn*/);
  }

  // Called every time the scheduler runs while the command is scheduled.
  double angleError;
  double distance;

  boolean speedOnTarget = false;
  boolean hoodOnTarget = false;
  boolean angleOnTarget = false;

  @Override
  public void execute() {
    if (!foundTarget) {
      distance = shooter.vision.getDistance();
      if (distance != 0.0) {
        foundTarget = true;
        shooterTargetSpeed = -shooter.calculateShooterSpeed(distance);  
        shooter.prepareShooter(distance);   
      }   
    }

    angleError = shooter.vision.getRobotAngle();

    //System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance) + "   Current Speed: " + shooter.getSpeed() + " ");

    if (currentControlMode == ControlMethod.SPIN_UP){ 

      if (shooter.speedOnTarget(shooterTargetSpeed, 15)) {
        if (startedTimerFlag) {
          if (Robot.time() - stableRPMTime * 1.0e-9 > 0.2) {
            currentControlMode = ControlMethod.HOLD;
          }
        }
        else {
          stableRPMTime = System.nanoTime();
          startedTimerFlag = true;
        }
      }
      else {
        startedTimerFlag = false;
      }
    }
    else if (currentControlMode == ControlMethod.HOLD) {
      if(setPid){
        pidTuner.HoldTune();
      }
      setPid = false;
    }

  
    speedOnTarget = (shooter.speedOnTarget(shooterTargetSpeed, 8) && currentControlMode == ControlMethod.HOLD) || Robot.time() - startTime * 1.0e-9 > 3.5; //TODO: May need to adjust acceptable error
    hoodOnTarget = Robot.time() - startTime * 1.0e-9 > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));

    if (speedOnTarget && hoodOnTarget)
        rapidFire();

  }

  // if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) && shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
  //   shooter.shoot();
  // } //The allowed error here matters a lot

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.vision.setMode(VisionMode.INTAKE);
    carousel.setBallCount(0);
    //carouselCommand.schedule();
    //if (rescheduleDriveCommand) {
     // driveCommand.schedule();
    //}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: this should just check to see if the carousel has rotated 5 CAROUSEL_FIFTH_ROTATION_TICKS intervals
    return (carousel.getTicks() - initialCarouselTicks) < -5 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS || (distance == 0.0 && Robot.time() - startTime * 1.0e-9 > 2.0);
            /*((double)System.nanoTime() - startTime) / 1_000_000_000.0 > 7.0;*/
    // if (waitTime == 0.0) {
    //   return false;
    // }
    // else {
    //   return ((System.nanoTime() - startTime) / 1_000_000_000.0 >= waitTime);
    // }
  }
}