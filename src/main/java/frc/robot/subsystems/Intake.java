/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.simulation.SparkMaxWrapper;

public class Intake extends SubsystemBase {
    SparkMaxWrapper intakeMotor;

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatch = new ColorMatch();

    private final Color blueTarget = new Color(0.143, 0.427, 0.429);
    private final Color greenTarget = new Color(0.197, 0.561, 0.240);
    private final Color redTarget = new Color(0.561, 0.232, 0.114);
    private final Color yellowTarget = new Color(0.361, 0.524, 0.113);

    public enum OutColor {
        RED, YELLOW, GREEN, BLUE, BAD
    }

    public Intake() {
        intakeMotor = new SparkMaxWrapper(Constants.INTAKE_MOTOR_CAN_ID , MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        
        //colorSensor = new ColorSensorV3(null);

        colorMatch.addColorMatch(blueTarget);
        colorMatch.addColorMatch(greenTarget);
        colorMatch.addColorMatch(redTarget);
        colorMatch.addColorMatch(yellowTarget);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        intakeMotor.set(-speed);
    }

    public OutColor read() {
        Color reading = colorSensor.getColor();
        // This converts no matter what, so it works for specific colors, and doesn't matter for rotations
        return convertColorFromReading(colorMatch.matchClosestColor(reading).color);
    }

    public void IntakeBalls() {
        run(0.25);
    }

    public void OutputBalls() {
        run(-0.25);
    }

    private OutColor convertColorFromReading (Color colorReading) {
        if (colorReading == redTarget) {
            return OutColor.GREEN;
        }
        else if (colorReading == yellowTarget) {
            return OutColor.RED;
        }
        else if (colorReading == blueTarget) {
            return OutColor.YELLOW;
        }
        else if (colorReading == greenTarget) {
            return OutColor.BLUE;
        }
        else {
            return OutColor.BAD;
        }
    }
}