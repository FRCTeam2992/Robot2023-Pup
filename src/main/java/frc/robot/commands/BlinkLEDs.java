// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.subsystems.LEDs;

public class BlinkLEDs extends CommandBase {
    private LEDs mLEDs;
    private IntakeModeState intakeMode;
    private int cyclesToDisplay = 5;

    private int ledsLoopCounter;

    /** Creates a new CycleLEDs. */
    public BlinkLEDs(LEDs leds, IntakeModeState intakeMode) {
        mLEDs = leds;
        this.intakeMode = intakeMode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mLEDs);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ledsLoopCounter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ledsLoopCounter == cyclesToDisplay) {
            switch (intakeMode) {
                case Cube:
                    mLEDs.displayCube();
                    break;
                case Cone:
                    mLEDs.displayCone();
                    break;
                case Unknown:
                    mLEDs.setLEDStripColor(Constants.LEDColors.blue);
                    break;
            }
        }
        if (ledsLoopCounter == 2 * cyclesToDisplay) {
            mLEDs.setLEDStripColor(Constants.LEDColors.off);
            ledsLoopCounter = 0;
        }
        ledsLoopCounter++;
    }

    // Called when the command is scheduled to run during disabled mode
    // Return of true allows the command to run
    @Override
    public boolean runsWhenDisabled() {
        return true;
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
