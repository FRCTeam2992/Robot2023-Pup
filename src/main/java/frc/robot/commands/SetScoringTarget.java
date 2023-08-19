// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;


public class SetScoringTarget extends CommandBase {
  private RobotState mRobotState;

  private BooleanSupplier isLowScore;
  private BooleanSupplier isBackScore;

  /** Creates a new SetScoringTargetXBox. */
  public SetScoringTarget(RobotState robotState, BooleanSupplier lowScoreSupplier,
      BooleanSupplier backScoreSupplier) {
    mRobotState = robotState;

    isLowScore = lowScoreSupplier;
    isBackScore = backScoreSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean low = isLowScore.getAsBoolean();
    boolean back = isBackScore.getAsBoolean();

    if (back) {
      if (low) {
        mRobotState.setTargetPosition(RobotState.GridTargetingPosition.LowBack);
      } else {
        mRobotState.setTargetPosition(RobotState.GridTargetingPosition.MidBack);
      }
    } else {
      if (low) {
        mRobotState.setTargetPosition(RobotState.GridTargetingPosition.LowFront);
      } else {
        mRobotState.setTargetPosition(RobotState.GridTargetingPosition.MidFront);
      }
    }

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
