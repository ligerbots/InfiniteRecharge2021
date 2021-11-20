
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

  CarouselCommand carouselCommand;
  //DriveCommand driveCommand;

  int initialCarouselTicks;

  double stableRPMTime;
  boolean startedTimerFlag;
  boolean foundTarget;
  boolean setPid;
 
  public enum ControlMethod {
    ACQUIRING, // Acquiring vision target
    SPIN_UP, // PIDF to desired RPM
    HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
  }

  ControlMethod currentControlMode;
  boolean rescheduleDriveCommand;

  public ShooterCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive, CarouselCommand carouselCommand, /*DriveCommand driveCommand,*/ boolean rescheduleDriveCommand) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.carousel = carousel;
    // The following statement will cause the CarouselCommand to be interrupted. This is good.
    // addRequirements(carousel);
    this.robotDrive = robotDrive;
    this.carouselCommand = carouselCommand;
    // System.out.println("Shooter.carouselCommand = " + this.carouselCommand);
    // this.driveCommand = driveCommand;
    this.rescheduleDriveCommand = rescheduleDriveCommand;
    startShooting = false;
    pidTuner = new ShooterPIDTuner(shooter);
  }

  public void rapidFire() {
    shooter.shoot();
    carousel.spin(Constants.CAROUSEL_SHOOTER_SPEED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("shooter/Shooting", "Shoot");

    foundTarget = false;
    shooterTargetSpeed = 0.0;
    // This flag is used so we only set the PID values once per command. We don't want to constantly reset the PID
    // values  in the execute() method.
    setPid = true;

    // driveCommand.cancel();
    startTime = Robot.time();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    carouselCommand.cancel();
    currentControlMode = ControlMethod.ACQUIRING;
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
    // shooter.setTurretAdjusted(0.0/*-Robot.angleErrorAfterTurn*/);
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
        currentControlMode = ControlMethod.SPIN_UP;
        // We found the target. Set the turret angle based on the vision system before
        // we spin up the shooter
        angleError = shooter.vision.getRobotAngle();
        // angleError = 0.0;
        shooter.setTurretAdjusted(angleError);
        shooterTargetSpeed = -shooter.calculateShooterSpeed(distance);  
        shooter.prepareShooter(distance);   
      }   
    }

    //System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance) + "   Current Speed: " + shooter.getSpeed() + " ");

    if (currentControlMode == ControlMethod.SPIN_UP){ 

      if (shooter.speedOnTarget(shooterTargetSpeed, 15)) {
        if (startedTimerFlag) {
          if (Robot.time() - stableRPMTime > 0.2) {
            currentControlMode = ControlMethod.HOLD;
          }
        } else {
          stableRPMTime = Robot.time();
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

  
    speedOnTarget = (shooter.speedOnTarget(shooterTargetSpeed, 8) && currentControlMode == ControlMethod.HOLD) || Robot.time() - startTime > 3.5; //TODO: May need to adjust acceptable error
    hoodOnTarget = Robot.time() - startTime > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));

    // !carousel.backwards will need to be removed when the shooter is re-written
    if (speedOnTarget && hoodOnTarget && !carousel.backwards)
        rapidFire();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.vision.setMode(VisionMode.INTAKE);
    carousel.spin(0.0);
    carousel.resetBallCount();
    carouselCommand.schedule();
    System.out.println("Shooter: carouselCommand scheduled" + carouselCommand);
    //if (rescheduleDriveCommand) {
     // driveCommand.schedule();
    //}
    SmartDashboard.putString("shooter/Shooting", "Idle");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.isSimulation()) return (Robot.time() - startTime) > 2.0;

    // TODO: this should just check to see if the carousel has rotated 5 CAROUSEL_FIFTH_ROTATION_TICKS intervals
    return (carousel.getTicks() - initialCarouselTicks) < -5 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS || (distance == 0.0 && Robot.time() - startTime > 2.0);
  }
}
