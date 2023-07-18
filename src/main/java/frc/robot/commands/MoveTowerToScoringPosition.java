// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.manipulator.Waypoint;
import frc.robot.RobotState;
import frc.robot.subsystems.Arm;

public class MoveTowerToScoringPosition extends CommandBase {
    private Arm mArm;
    private RobotState mRobotState;

    /** Creates a new MoveTowerToScoringPosition. */
    public MoveTowerToScoringPosition(Arm arm, RobotState robotState) {
        mArm = arm;
        mRobotState = robotState;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Waypoint waypoint = mRobotState.currentTargetPosition.towerWaypoint;
        mRobotState.currentOuttakeType = waypoint.outtakeType();

        CommandScheduler.getInstance().schedule(
                new SetArmPosition(mArm, waypoint.angle()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
