// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FinishIntakeSequence extends SequentialCommandGroup {
  /** Creates a new FinishIntakeSequence. */
  public FinishIntakeSequence(Elevator e, Arm a, Claw c, Intake i, IntakeDeploy id, Spindexer s) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetClawState(c, ClawState.Opened),
        new ParallelRaceGroup(
            new MoveIntake(i, 0, 0),
            new SetIntakeDeployState(id, IntakeDeployState.Homed),
            new SafeDumbTowerToPosition(e, a, Constants.TowerConstants.intakeGrab)).withTimeout(1.5),
        // new AutoSpinSpindexer(s).repeatedly().withTimeout(1.5)),
        new SetClawState(c, ClawState.Closed)

    );

  }
}
