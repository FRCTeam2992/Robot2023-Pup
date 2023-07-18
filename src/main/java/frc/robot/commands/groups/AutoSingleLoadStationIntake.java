// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSingleLoadStationIntake extends ParallelCommandGroup {
    /** Creates a new AutoLoadStationIntake. */
    public AutoSingleLoadStationIntake(Arm arm, Claw claw, LEDs leds, RobotState robotState) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetArmPosition(arm, Constants.TowerConstants.singleLoadStation.angle()),
            new InstantCommand(() -> {
                robotState.currentOuttakeType = OuttakeType.Unknown;
                if (robotState.intakeMode == RobotState.IntakeModeState.Unknown) {
                    robotState.intakeMode = RobotState.IntakeModeState.Cube;
                }
            }).andThen(new IntakeGamePiece(claw, leds, robotState)));
    }
}
