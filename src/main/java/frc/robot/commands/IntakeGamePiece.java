// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LEDs;

public class IntakeGamePiece extends CommandBase {
    private Claw mClaw;
    private LEDs mLEDs;
    private RobotState mRobotState;

    private int cyclesAfterBeamBreak;
    private double speed;

    /** Creates a new IntakeGamePiece. */
    public IntakeGamePiece(Claw claw, LEDs leds, RobotState robotState) {
        mClaw = claw;
        mLEDs = leds;
        mRobotState = robotState;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mClaw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cyclesAfterBeamBreak = 0;
        speed = 0.0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (mRobotState.intakeMode) {
            case Cone:
                speed = ClawConstants.Intake.Speed.cone;
                break;
            case Cube:
            case Unknown:
            default:
                speed = ClawConstants.Intake.Speed.cube;
        }
        mClaw.setClawSpeed(speed);
        if (mClaw.getBeamBreakTriggered()) {
            cyclesAfterBeamBreak++;
        } else {
            cyclesAfterBeamBreak = 0;
        }
        if (Constants.debugDashboard) {
            // Troubleshooting only dashboard
            SmartDashboard.putNumber("Beam Break Cycles", cyclesAfterBeamBreak);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mClaw.setClawSpeed(0.0);
        if (mClaw.getBeamBreakTriggered()) {
            CommandBase cmd =  new BlinkLEDs(mLEDs, mRobotState.intakeMode).withTimeout(1.0);

            switch (mRobotState.intakeMode) {
                case Cone:
                    cmd = cmd.andThen(new SetLEDsCone(mLEDs));
                    break;
                case Cube:
                case Unknown:
                default:
                    cmd = cmd.andThen(new SetLEDsCube(mLEDs));
            }
            CommandScheduler.getInstance().schedule(cmd);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        switch (mRobotState.intakeMode) {
            case Cube:
                return cyclesAfterBeamBreak >= ClawConstants.Intake.DelayCyclesAfterBeamBreak.cube;
            case Cone:
            case Unknown:
            default:
                return cyclesAfterBeamBreak >= ClawConstants.Intake.DelayCyclesAfterBeamBreak.cone;
        }
    }
}
