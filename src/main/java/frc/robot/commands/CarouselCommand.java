package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;

public class CarouselCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  double lastTimeCheck;
  double timeCheck;
  boolean backwards;

  double lastBackTime;

  int currentTicks;
  //int lastCheckpoint;
  //int currentCheckpoint;

  boolean stopForOpenSpace;

  final int fifthRotationTicks = Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
  final double pauseTime = 0.04; // seconds

  public CarouselCommand(Carousel carousel) {
    this.carousel = carousel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeCheck = Robot.time();
    lastTimeCheck = timeCheck;
    currentTicks = 0;
    //lastCheckpoint = 0;
    //currentCheckpoint = 0;
    backwards = false;
    stopForOpenSpace = !carousel.isBallInFront();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Color Sensor distance reading", carousel.getColorSensorProximity());
    SmartDashboard.putNumber("Output current", carousel.getCurrent());
    SmartDashboard.putNumber("Carousel ticks", carousel.getTicks());

    currentTicks = -carousel.getTicks();
    Carousel.currentCheckpoint = currentTicks / fifthRotationTicks;

    if (carousel.getCurrent() > 10.35) { // First check is to see if the current is spiking
      lastBackTime = Robot.time(); // start timer for going backwards
      backwards = true; // now we goin backwards
    }

    if (!backwards) { // This is what we do if we aren't going backwards
      if (Carousel.currentCheckpoint > Carousel.lastCheckpoint) { // if we have indexed to the next slot...
        Carousel.lastCheckpoint = Carousel.currentCheckpoint;
        lastTimeCheck = Robot.time(); // Start the timer for pausing at a slot
      }
      if (Robot.time() - lastTimeCheck > pauseTime && !stopForOpenSpace/* && Carousel.checkForFullnessCount < 5*/) {
        // This block executes if we aren't pausing, the slot isn't open, and we haven't already gone around 5 times
        carousel.spin(Constants.CAROUSEL_INTAKE_SPEED); // Spin the carousel
      }
      else { // This block runs if 
        if (!carousel.isBallInFront()) { // This block runs if there is not ball up front
          stopForOpenSpace = true; 
          Carousel.checkForFullnessCount = 0; // reset the counter to see if we are full, cause we obviously aren't
        }
        else {
          stopForOpenSpace = false;
          Carousel.checkForFullnessCount += 1;
        }
        carousel.spin(0); // We aren't supposed to be spinning here
      }
    }
    else { // This is decently readable go backwards code that runs on a timer
      carousel.spin(-0.6);
      if (Robot.time() - lastBackTime > 0.5) {
        backwards = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}