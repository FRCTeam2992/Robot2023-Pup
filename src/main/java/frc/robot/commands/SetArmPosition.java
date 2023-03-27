// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {
    /** Creates a new SetArmPosition. */
    private Arm mArm;
    private double mAngle;
    private int thereCounter = 0;

    public SetArmPosition(Arm subsystem, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        mArm = subsystem;
        mAngle = angle;

        addRequirements(mArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mArm.setArmTarget(mAngle);
        thereCounter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // PID loop runs in subsystem periodic
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mArm.setArmSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mArm.atPosition();
    }
}
