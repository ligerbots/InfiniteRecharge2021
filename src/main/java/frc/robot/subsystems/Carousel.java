/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Carousel extends SubsystemBase {
    WPI_TalonSRX spinner;
    Encoder carouselEncoder;
    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    int ballCount = 0;
    double lastSpinSpeed;
    // public until shooter is updated
    public boolean backwards;
    double lastBackTime;

    public Carousel() {
        spinner = new WPI_TalonSRX(Constants.CAROUSEL_CAN_ID);
        spinner.setNeutralMode(NeutralMode.Brake);
        carouselEncoder = new Encoder(7, 8);
        backwards = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Carousel/Color Sensor distance", getColorSensorProximity());
        SmartDashboard.putNumber("Carousel/Output current", getCurrent());
        SmartDashboard.putNumber("Carousel/Ticks", -getTicks());
        SmartDashboard.putNumber("Carousel/Slot", getSlot());

        // First check is to see if the current is spiking, this would indicate a stuck ball
        if (getCurrent() > 10.35) { 
            lastBackTime = Robot.time(); // start timer for going backwards
            backwards = true; // now we goin backwards
            // set the spinner going backwards to unjam the ball
            // do NOT use the public spin() method, since we do not want this speed remembered
            spinner.set(ControlMode.PercentOutput, -0.6);
        }
        
        if (backwards && Robot.time() - lastBackTime > 0.5) {
            // sets carousel to operate normally again if the carousel has been running 
            // in reverse for half a second
            backwards = false;
            spin(lastSpinSpeed);
        } 
    }

    public void spin(double speed) {
        spinner.set(ControlMode.PercentOutput, speed);
        lastSpinSpeed = speed;
    }

    public double getCurrent () {
        return spinner.getStatorCurrent();
    }

    public void warmUp() {
        this.spin(Constants.CAROUSEL_SHOOTER_SPEED);
    }
    
    public int getTicks() {
        return carouselEncoder.getRaw();
    }

    public double getSlot() {
        // because of the encoder position, the returned value is negative
        // so the encoder output must be made negative to return a positive encoder value
        return (double)(-getTicks()) / Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
    }

    public void resetEncoder () {
        carouselEncoder.reset();
    }

    public boolean isBallInFront () {
        return colorSensor.getProximity() > 110;
    }

    public int getColorSensorProximity() {
        return colorSensor.getProximity();
    }

    public void incrementBallCount () {
        ballCount++;
    }

    public void resetBallCount () {
        ballCount = 0;
    }

    public int getBallCount () {
        return ballCount;
    }
}
