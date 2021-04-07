
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionMode;

public class ShooterOneCommand extends CommandBase {
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
  // DriveCommand driveCommand;

  int initialCarouselTicks;

  double stableRPMTime;
  boolean startedTimerFlag;
  boolean foundTarget;
  boolean setPid;

  public enum ControlMethod {
    ACQUIRING, // Acquiring vision target
    SPIN_UP, // PIDF to desired RPM
    // HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
    STEP_CAROUSEL
  }

  ControlMethod currentControlMode;
  boolean rescheduleDriveCommand;

  double targetSlot;
  final double earlySlotStopDelta = 1830.0 / Constants.CAROUSEL_FIFTH_ROTATION_TICKS;

  public ShooterOneCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive,
      /* double waitTime, */ CarouselCommand carouselCommand,
      /* DriveCommand driveCommand, */ boolean rescheduleDriveCommand) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.carousel = carousel;
    // The following statement will cause the CarouselCommand to be interrupted.
    // This is good.
    // addRequirements(carousel);
    this.robotDrive = robotDrive;
    // this.waitTime = waitTime;
    this.carouselCommand = carouselCommand;
    // System.out.println("Shooter.carouselCommand = " + this.carouselCommand);
    // this.driveCommand = driveCommand;
    this.rescheduleDriveCommand = rescheduleDriveCommand;
    startShooting = false;
    pidTuner = new ShooterPIDTuner(shooter);
  }

  // public void rapidFire() {
  // shooter.shoot();
  // // carousel.spin(Constants.CAROUSEL_SHOOTER_SPEED);
  // carousel.spin(Constants.CAROUSEL_GS_SHOOTER_SPEED);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("shooter/Shooting", "Shoot");

    foundTarget = false;
    shooterTargetSpeed = 0.0;
    // This flag is used so we only set the PID values once per command. We don't
    // want to constantly reset the PID
    // values in the execute() method.
    setPid = true;

    // driveCommand.cancel();
    startTime = Robot.time();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    carouselCommand.cancel();
    currentControlMode = ControlMethod.ACQUIRING;
    // starts spinning up the shooter to hard-coded PID values
    pidTuner.spinUpTune();
    System.out.println("Initial NavX Heading: " + robotDrive.getHeading());
    // store current carouselTick value
    initialCarouselTicks = carousel.getTicks();

    // angleError = shooter.vision.getRobotAngle();
    // distance = shooter.vision.getDistance();

    // currentControlMode = ControlMethod.SPIN_UP;
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
    SmartDashboard.putString("shooter/ShootOneCmd", currentControlMode.toString());
    
    switch (currentControlMode) {
      case ACQUIRING:
        if (!foundTarget) {
          distance = shooter.vision.getDistance();
          if (distance != 0.0) {
            foundTarget = true;
            currentControlMode = ControlMethod.SPIN_UP;
            System.out.println("Shooter SPIN_UP");
            // We found the target. Set the turret angle based on the vision system before
            // we spin up the shooter
            angleError = shooter.vision.getRobotAngle();
            // angleError = 0.0;
            if (distance > 200) {
              angleError -= 1.0;
            } else if (distance <95) {
              angleError += 1.0;
            }
            // angleError = 0.0;
            SmartDashboard.putNumber("ShooterCmd/angleError", angleError);
            shooter.setTurretAdjusted(angleError);
            shooterTargetSpeed = -shooter.calculateShooterSpeed(distance);
            shooter.prepareShooter(distance);
          }
        }
        break;

      // System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance)
      // + " Current Speed: " + shooter.getSpeed() + " ");

      case SPIN_UP:
        // if (currentControlMode == ControlMethod.SPIN_UP){

        if (shooter.speedOnTarget(shooterTargetSpeed, 5)) {
          if (startedTimerFlag) {
            if (Robot.time() - stableRPMTime > 1.0) {
              currentControlMode = ControlMethod.HOLD;
              // HOLD restarts its own timer
              startedTimerFlag = false;
              System.out.println("Shooter HOLD");
            }
          } else {
            stableRPMTime = Robot.time();
            startedTimerFlag = true;
          }
        } else {
          startedTimerFlag = false;
        }
        break;
        // }

        // Hold until we're within 5% for a whole second
        // else if (currentControlMode == ControlMethod.HOLD) {
        // if(setPid){
        // Commented out the following line. Want to keep PID tuner and not reset
        // accumulators
        // pidTuner.HoldTune();
        // }
        // setPid = false;
        // }
      case HOLD:
        if (shooter.speedOnTarget(shooterTargetSpeed, 5)) {
          // If shooter is at speed, start timer
          System.out.println("Shooter waiting");
          if (! startedTimerFlag) {
            stableRPMTime = Robot.time();
            startedTimerFlag = true;
          }
        } else {
          // Need to restart timer
          startedTimerFlag = false;
        }

        // speedOnTarget = (shooter.speedOnTarget(shooterTargetSpeed, 5) &&
        // currentControlMode == ControlMethod.HOLD) || Robot.time() - startTime > 3.5;
        // //TODO: May need to adjust acceptable error
        speedOnTarget = Robot.time() - stableRPMTime > 1.0;
        hoodOnTarget = Robot.time() - startTime > 0.75;// shooter.hoodOnTarget(shooter.calculateShooterHood(distance));

        // !carousel.backwards will need to be removed when the shooter is re-written
        if (speedOnTarget && hoodOnTarget && !carousel.backwards) {

          currentControlMode = ControlMethod.STEP_CAROUSEL;
          System.out.println("Shooter STEP_CAROUSEL");
          // set target slot so carousel only moves one slot
          targetSlot = Math.round(carousel.getSlot()) + 1.0 - earlySlotStopDelta;
          // start the flup
          shooter.shoot();
        }
        break;

      case STEP_CAROUSEL:
        if (carousel.getSlot() >= targetSlot) {
          // Stop Carousel and then get ready to shoot again
          carousel.spin(0.0);
          currentControlMode = ControlMethod.SPIN_UP;
        } else {
          carousel.spin(Constants.CAROUSEL_GS_SHOOTER_SPEED);
        }
        break;

      default:
        break;
    }
  }
  // if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) &&
  // shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
  // shooter.shoot();
  // } //The allowed error here matters a lot

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    carousel.spin(0.0);
    shooter.vision.setMode(VisionMode.INTAKE);
    carousel.resetBallCount();
    carouselCommand.schedule();
    System.out.println("Shooter: carouselCommand scheduled" + carouselCommand);
    // if (rescheduleDriveCommand) {
    // driveCommand.schedule();
    // }
    SmartDashboard.putString("shooter/Shooting", "Idle");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.isSimulation())
      return (Robot.time() - startTime) > 2.0;

    // TODO: this should just check to see if the carousel has rotated enough
    // CAROUSEL_FIFTH_ROTATION_TICKS intervals or we never found the target
    return (carousel.getTicks() - initialCarouselTicks) < -Constants.CAROUSEL_MAX_BALLS * Constants.CAROUSEL_FIFTH_ROTATION_TICKS
        || (currentControlMode == ControlMethod.ACQUIRING && Robot.time() - startTime > 2.0);
  }
}
