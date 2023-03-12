// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;
import java.security.spec.EncodedKeySpec;
import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.math.MathUtil;

public class Arm extends PIDSubsystem {
    public TrapezoidProfile profile;
    public CANSparkMax winch;
    public PIDController controllerD;
    public AnalogPotentiometer potentiometer;

    /** Creates a new ExampleSubsystem. */
    public Arm(CANSparkMax winch, AnalogPotentiometer potentiometer, PIDController controllerD) {
        super(controllerD);
        this.controllerD = controllerD;
        this.winch = winch;
        this.potentiometer = potentiometer; 
    }

    @Override
    public void useOutput(double current, double setpoint) {
        // Calculate the feedforward from the sepoint
        //double feedforward = feedforward.calculate(current, setpoint);
        // Add the feedforward to the PID output to get the motor output
        profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(5, 10),
        new TrapezoidProfile.State(getMeasurementD(), getMeasurement()),
        new TrapezoidProfile.State(setpoint, 0));

        double out = controllerD.calculate(potentiometer.get(), setpoint);

        double outFiltered = MathUtil.clamp(out, -8, 8);
        winch.setVoltage(outFiltered);
    }

    public double getMeasurementD() {
        return potentiometer.get();
    }

    public double getMeasurement() {
        return winch.get();
    }

    public void raise() {
        if (potentiometer.get() < 90) {
            winch.set(.5);
        }
    }

    public void lower() {
        if (potentiometer.get() > 0) {
            winch.set(-.5);
        }
    }

    public void stop() {
        winch.set(0);
    }
}
