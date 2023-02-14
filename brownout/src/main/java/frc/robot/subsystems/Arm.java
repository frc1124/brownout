// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static CANSparkMax winchOne;
    private static CANSparkMax winchTwo;
    /** Creates a new ExampleSubsystem. */
    public Arm() {
        winchOne = new CANSparkMax(Constants.ARM_STAGE_ONE, MotorType.kBrushless);
        winchTwo = new CANSparkMax(Constants.ARM_STAGE_TWO, MotorType.kBrushless);
    }

    public void OpenOne() {
        winchOne.set(1);
    }

    public void CloseOne() {
        winchOne.set(-1);
    }

    public void OpenTwo() {
        winchTwo.set(1);
    }

    public void CloseTwo() {
        winchTwo.set(-1);
    }
}
