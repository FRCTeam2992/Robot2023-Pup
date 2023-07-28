// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class SetLEDsCone extends CommandBase {
  private LEDs mLEDs;

  /** Creates a new SetLEDsCone. */
  public SetLEDsCone(LEDs subsystem) {
    mLEDs = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLEDs.displayCone();
    System.out.println("///////////////////////////////////////// Displaying Cone");
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
    return false;
  }
}
