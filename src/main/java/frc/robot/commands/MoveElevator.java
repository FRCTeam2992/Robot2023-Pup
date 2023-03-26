// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.RobotState;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase {
    /** Creates a new MoveElevator. */
    private Elevator mElevator;
    private RobotState mRobotState;

    private double mElevatorSpeed;

    public MoveElevator(Elevator subsystem, RobotState robotState, double elevatorspeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        mElevator = subsystem;
        mRobotState = robotState;
        mElevatorSpeed = elevatorspeed;
        addRequirements(mElevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mElevator.setElevatorSpeed(mElevatorSpeed);
        // mRobotState.currentOuttakeType = OuttakeType.Unknown;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mElevator.setElevatorSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
