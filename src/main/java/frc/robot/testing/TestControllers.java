package frc.robot.testing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveArm;

public class TestControllers {
    // private final CommandXboxController testController0;
    private final CommandXboxController testController1;

    public TestControllers() {
        // testController0 = new CommandXboxController(2);
        testController1 = new CommandXboxController(3);
    }

    public void configTestButtonBindings(RobotContainer mRobotContainer) {
        /*
         * DO NOT USE "controller0" or "controller1" here
         */
        testController1.povLeft().whileTrue(new MoveArm(mRobotContainer.mArm, .1));
        testController1.povRight().whileTrue(new MoveArm(mRobotContainer.mArm, -.1));

        testController1.leftBumper().onTrue(new InstantCommand(() -> {
            mRobotContainer.mDrivetrain.setInSlowMode(true);
        }));
        testController1.leftBumper().onFalse(new InstantCommand(() -> {
            mRobotContainer.mDrivetrain.setInSlowMode(false);
        }));
    }
}
