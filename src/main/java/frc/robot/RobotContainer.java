// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.autonomous.AutoBuilder;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.BalanceRobotPID;
import frc.robot.commands.ClawOuttake;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.HoldArm;
import frc.robot.commands.HoldClaw;
import frc.robot.commands.LEDsToDefaultColor;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmToPoint;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.MoveTowerToScoringPosition;
import frc.robot.commands.SetSwerveAngle;

import frc.robot.commands.SetLEDsColor;
import frc.robot.commands.SetLEDsCone;
import frc.robot.commands.SetLEDsCube;
import frc.robot.commands.SetScoringTarget;
import frc.robot.commands.ToggleEndgameState;
import frc.robot.commands.groups.AutoGroundIntakeCube;
import frc.robot.commands.groups.AutoSingleLoadStationIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.LEDs;
import frc.robot.testing.commands.TestArmPID;
import frc.robot.testing.commands.TestClawIntake;
import frc.robot.testing.commands.TestClawOuttake;
import frc.robot.testing.commands.TestArmMove;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the
 * robot (including subsystems, commands, and trigger mappings) should
 * be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    public final RobotState mRobotState;
    public final AutoBuilder mAutoBuilder;

    public final Drivetrain mDrivetrain;

    public final Arm mArm;
    public final Claw mClaw;

    public final LEDs mLEDs;

    public final PowerDistribution pdp;

    public DigitalInput networkToggleSwitch = new DigitalInput(
            Constants.RobotConstants.DeviceIDs.networkToggleSwitch);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        mRobotState = new RobotState();

        mDrivetrain = new Drivetrain(mRobotState);
        mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain, mRobotState));

        mArm = new Arm();
        // mArm.setDefaultCommand(new StopArm(mArm));
        mArm.setDefaultCommand(new HoldArm(mArm));

        mClaw = new Claw();
        mClaw.setDefaultCommand(new HoldClaw(mClaw));
        // mClaw.setDefaultCommand(new StopClaw(mClaw));

        mLEDs = new LEDs();
        mLEDs.setDefaultCommand(new SetLEDsColor(mLEDs, Constants.LEDColors.off));

        mAutoBuilder = new AutoBuilder(mRobotState, mDrivetrain, mArm, mClaw, mLEDs);

        // Setup the Auto Selectors
        mAutoBuilder.setupAutoSelector();

        pdp = new PowerDistribution(0, ModuleType.kCTRE);

        // Add dashboard things
        addSubsystemsToDashboard();
        addRobotStateToDashboard();
        updateMatchStartChecksToDashboard();

        // Configure the trigger bindings
        configureShuffleboardBindings();
        configRealButtonBindings();
        // (new TestControllers()).configTestButtonBindings(this);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * t * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link
     * edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */

    private void configRealButtonBindings() {
        /*
         * DO NOT PUT TEST BUTTONS IN THIS
         * ONLY REAL BUTTONS FOR COMPETITION
         */

        // -----------------------controller0-----------------------

        // ABXY
        // A = auto-align for scoring
        controller0.a().onTrue(new InstantCommand(() -> {
            mDrivetrain.setScoringMode(true);
        }));
        controller0.a().onFalse(new InstantCommand(() -> {
            mDrivetrain.setScoringMode(false);
        }));

        // B = intake from load station
        controller0.b().onTrue(
                new AutoSingleLoadStationIntake(mArm, mClaw, mLEDs, mRobotState));
        controller0.b().onTrue(new InstantCommand(() -> {
            mDrivetrain.setLoadingMode(true);
        }));
        controller0.b().onFalse(new InstantCommand(() -> {
            mDrivetrain.setLoadingMode(false);
        }));

        // X = ground intake cube
        controller0.x().onTrue(
                new AutoGroundIntakeCube(mArm, mClaw, mLEDs, mRobotState));// cubes
        controller0.x().onTrue(new SetLEDsCube(mLEDs));

        // D-Pad
        controller0.povLeft().whileTrue(mDrivetrain.XWheels());// X the wheels

        controller0.povUp().onTrue(new SetLEDsCone(mLEDs));
        controller0.povUp()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cone));
        controller0.povDown().onTrue(new SetLEDsCube(mLEDs));
        controller0.povDown()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cube));

        // Bumpers/Triggers
        controller0.leftBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(false);
                }));// Disable Field Orient
        controller0.leftBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(true);
                }));// Disable Field Orient

        controller0.rightBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(true);
                })); // Slow Mode
        controller0.rightBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(false);
                })); // Slow Mode

        controller0.leftTrigger(0.6)
                .onTrue(new MoveTowerToScoringPosition(mArm, mRobotState));
        controller0.leftTrigger(0.6)
                .onFalse(new InstantCommand(() -> mRobotState.currentOuttakeType = OuttakeType.Unknown)
                        .andThen(new SetArmPosition(mArm, TowerConstants.normal.angle())));

        controller0.rightTrigger(0.6)
                .onTrue(new ClawOuttake(mClaw, mRobotState));

        controller0.rightStick().onTrue(new SetArmPosition(mArm, Constants.TowerConstants.normal.angle()));
        controller0.rightStick().onTrue(new HoldClaw(mClaw));

        // Back and Start

        controller0.start().onTrue(new ResetGyro(mDrivetrain));

        controller0.back().onTrue(new BalanceRobotPID(mDrivetrain));

        // Joysticks Buttons

        // -----------------------controller1-----------------------
        switch (Constants.controller1Mode) {

            case XBox:
                configureController1XBoxMappings();
                break;

            case Guitar:
                configureController1GuitarMappings();
                break;
        }

    }

    private void configureController1GuitarMappings() {
        // Color Neck Buttons

        // Green/A
        controller1.a().onTrue(new SetScoringTarget(mArm, mRobotState, () -> true,
                        () -> controller1.leftTrigger(0.6).getAsBoolean()));

        // Red/B
        controller1.b().onTrue(new SetScoringTarget(mArm, mRobotState, () -> false,
                        () -> controller1.leftTrigger(0.6).getAsBoolean()));

        // Yellow/Y
        controller1.y().onTrue(new SetLEDsCone(mLEDs));
        controller1.y()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cone));
        // Blue/X
        controller1.x().onTrue(new SetLEDsCube(mLEDs));
        controller1.x()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cube));

        // Orange/LB

        controller1.leftBumper().onTrue(new SetArmPosition(mArm, Constants.TowerConstants.normal.angle()));
        controller1.leftBumper().onTrue(new HoldClaw(mClaw));

        // Strummer

        controller1.povDown().whileTrue(new MoveClaw(mClaw, -0.8));
        controller1.povUp().whileTrue(new MoveClaw(mClaw, 0.8));


        // -+/Body buttons (+ = start; - = back)

        controller1.start().whileTrue(new MoveArm(mArm, 0.2));
        controller1.back().whileTrue(new MoveArm(mArm, -0.2));

        // Wammy Bar



        // Joystick (Maps to left stick and POV Simultaneously)
    }

    private void configureController1XBoxMappings() {
            // XYAB

        controller1.y().whileTrue(new MoveClaw(mClaw, Constants.ClawConstants.Intake.Speed.cone));

        controller1.x().whileTrue(new MoveClaw(mClaw, -Constants.ClawConstants.Intake.Speed.cone));

        // Strummer
        controller1.leftBumper().onTrue(new SetLEDsCube(mLEDs));
        controller1.leftBumper()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cube));
        controller1.rightBumper().onTrue(new SetLEDsCone(mLEDs));
        controller1.rightBumper()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cone));

        controller1.leftTrigger(0.6).onTrue(new SetArmPosition(mArm, Constants.TowerConstants.normal.angle()));
        controller1.leftTrigger(0.6).onTrue(new HoldClaw(mClaw));

        controller1.rightTrigger(0.6)
                        .onTrue(new SetScoringTarget(mArm, mRobotState, () -> controller1.povDown().getAsBoolean(),
                                        () -> controller1.b().getAsBoolean()));

        // -+/Body buttons


        // Wammy Bar
        controller1.axisLessThan(XboxController.Axis.kLeftY.value, -0.6).whileTrue(
                new MoveArm(mArm, -0.40));
        controller1.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.6).whileTrue(
                        new MoveArm(mArm, 0.40));
    }

    private void configureShuffleboardBindings() {
        SmartDashboard.putData("Arm To Point 100%, 10°",
                new MoveArmToPoint(mArm, mClaw, mDrivetrain, 1.0, 10.0, 30.0, -5.0));
        if (Constants.debugDashboard) {
            // SmartDashboard.putData("Re-init Arm Encoder", new InstantCommand(() ->
            // mArm.initArmMotorEncoder()));

            // SmartDashboard.putData("Intake Game Piece", new IntakeGamePiece(mClaw,
            // mRobotState));

            SmartDashboard.putNumber("Test Claw Cube In Spd %", 0.5);
            SmartDashboard.putNumber("Test Claw Cone In Spd %", 0.7);
            SmartDashboard.putData("Test Claw Intake", new TestClawIntake(mClaw, mRobotState));

            SmartDashboard.putNumber("Test Claw Cube Out Spd %", 0.7);
            SmartDashboard.putNumber("Test Claw Cone Out Spd %", 0.5);
            SmartDashboard.putData("Test Claw Outtake", new TestClawOuttake(mClaw, mRobotState));

            SmartDashboard.putNumber("LED Current Draw", pdp.getCurrent(6));
            SmartDashboard.putNumber("Claw Current Draw", pdp.getCurrent(7));

            // SmartDashboard.putData("Reset Odometry to Red Inner Cone",
            // new InstantCommand(() -> mDrivetrain
            // .resetOdometryToPose(new Pose2d(1.89, 3.0307,
            // Rotation2d.fromDegrees(0.0)))));
            SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));

            // SmartDashboard.putData("Test Path Planner Path",
            // new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

            // SmartDashboard.putData("Test Path Planner Path",
            // new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

            SmartDashboard.putNumber("ArmTestMoveAngle", 0.0);
            SmartDashboard.putData("Test Move Arm", new TestArmMove(mArm));
            SmartDashboard.putData("Test PID Move Arm", new TestArmPID(mArm));
            SmartDashboard.putData("turn Off LEDS", new SetLEDsColor(mLEDs, Constants.LEDColors.off));

            SmartDashboard.putData("Set LEDs Cone Mode", new SetLEDsCone(mLEDs));
            SmartDashboard.putData("Set LEDs Cube Mode", new SetLEDsCube(mLEDs));
            SmartDashboard.putData("Turn LEDs Blue", new SetLEDsColor(mLEDs, Constants.LEDColors.blue));
            SmartDashboard.putData("Turn Off LEDS", new SetLEDsColor(mLEDs, Constants.LEDColors.off));
            SmartDashboard.putData("Max LEDS", new SetLEDsColor(mLEDs, Constants.LEDColors.max));

            // SmartDashboard.putData("TestAutoBalance", new BalanceRobot(mDrivetrain));
        }

        SmartDashboard.putData("Reset Odometry", mDrivetrain.ResetOdometry());
    }

    public void addSubsystemsToDashboard() {
        SmartDashboard.putData("Drivetrain", mDrivetrain);
        SmartDashboard.putData("Arm", mArm);
        SmartDashboard.putData("Claw", mClaw);
        SmartDashboard.putData("LEDs", mLEDs);
    }

    public void addRobotStateToDashboard() {

            SmartDashboard.putBoolean("Target: Low Back",
                            mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowBack);

            SmartDashboard.putBoolean("Target: Low Front",
                            mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowFront);

            SmartDashboard.putBoolean("Target: Mid Back",
                            mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidBack);

            SmartDashboard.putBoolean("Target: Mid Front",
                            mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidFront);

        if (Constants.debugDashboard) {
            SmartDashboard.putBoolean("Blue Alliance",
                    DriverStation.getAlliance() == DriverStation.Alliance.Blue);
            SmartDashboard.putBoolean("Red Alliance",
                    DriverStation.getAlliance() == DriverStation.Alliance.Red);

            SmartDashboard.putBoolean("Endgame Mode",
                    mRobotState.endgameMode == RobotState.EndgameModeState.InEndgame);
        }
    }

    public void updateMatchStartChecksToDashboard() {
        SmartDashboard.putString("Confirmed Auto Start Position",
                mAutoBuilder.getAutoStartPosition().description);
        if (mAutoBuilder.autoStartCompatible()) {
            SmartDashboard.putString("Confirmed Auto Sequence", mAutoBuilder.getAutoSequence().description);
        } else {
            SmartDashboard.putString("Confirmed Auto Sequence", "INVALID SEQUENCE FOR THIS START POSN");
        }
        SmartDashboard.putString("Confirmed Auto Preload Score",
                mAutoBuilder.getAutoPreloadScore().description);
        SmartDashboard.putBoolean("Valid Auto Sequence?", mAutoBuilder.autoStartCompatible());
    }

    public CommandXboxController getController0() {
        return controller0;
    }
}
