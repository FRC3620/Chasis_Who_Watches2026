// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.ChudbotTunerConstants;
import frc.robot.generated.JoeHannTunerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import gg.questnav.questnav.QuestNav;

public class RobotContainer {
    private double MaxSpeed = ChudbotTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                         // speed
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static CommandXboxController joystick = new CommandXboxController(0);

    public static CommandSwerveDrivetrain drivetrain;

    public static QuestNavSubsystem questNavSubsystem;

    public static LimelightSubsystem limelightSubsystem;

    //public final TurretSubsystem turretSubsystem = new TurretSubsystem();

    private SendableChooser<Command> autoChooser;

    public final TurretSubsystem turretSubsystem = new TurretSubsystem();

    public RobotContainer() {

        makeSubsystems();

        // questNavSubsystem = new QuestNavSubsystem(drivetrain,
        // autoChooser.getSelected());

        configureBindings();

        setupPathPlannerCommands();

        setUpAutonomousCommands();

        FollowPathCommand.warmupCommand().schedule();
    }

    public void makeSubsystems(){
        drivetrain = configureSwerveDrive();
        questNavSubsystem = new QuestNavSubsystem(drivetrain, new Pose3d());
        limelightSubsystem = new LimelightSubsystem();
      
        turretSubsystem.setDefaultCommand(turretSubsystem.setAngle(Degrees.of(0)));

    }

    private CommandSwerveDrivetrain configureSwerveDrive() {
        String serialNumber = RobotController.getSerialNumber();

        if (serialNumber.equals("0326F30D") || serialNumber.equals("0327BA54")) {
            SmartDashboard.putString("Robot Serial", serialNumber);
            SmartDashboard.putString("Robot Variant", "Chudbot");
            return ChudbotTunerConstants.createDrivetrain();
        } else if (serialNumber.equals("03063D7E")) {
            SmartDashboard.putString("Robot Serial", serialNumber);
            SmartDashboard.putString("Robot Variant", "JoeHann");
            return JoeHannTunerConstants.createDrivetrain();
        } else {
            SmartDashboard.putString("Robot Serial", serialNumber);
            SmartDashboard.putString("Robot Variant", "Other");
            return TunerConstants.createDrivetrain();
        }
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(), 0.2) * MaxSpeed) // Drive
                                                                                                                // forward
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // Y
                                                                                                                // (forward)
                                .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), 0.2) * MaxSpeed) // Drive
                                                                                                             // left
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // X (left)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

      //fix questnav correction command
        CommandScheduler.getInstance().schedule(new SetQuestNavPoseFromMegaTag1Command());
        CommandScheduler.getInstance().schedule(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(limelightSubsystem.getMegaTag1Rotation().getDegrees()))
            .andThen(new InstantCommand(() -> drivetrain.seedFieldCentric(limelightSubsystem.getMegaTag1Rotation()))));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drivemotors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(questNavSubsystem.zeroQuestNavPoseCommand());

        joystick.y().onTrue(new SetQuestNavPoseFromMegaTag1Command());

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povUp().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(0.2) // Drive forward with negative Y (forward)
                        .withVelocityY(0) // Drive left with negative X (left)
                        .withRotationalRate(0) // Drive coun
                ));

    }

    public void setUpAutonomousCommands() {
        if (drivetrain != null) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = null;
        }

        if (autoChooser != null) {
            SmartDashboard.putData("Auto Mode", autoChooser);
        }
    }

    public Command getAutonomousCommand() {
        if (autoChooser != null) {
            return autoChooser.getSelected();
        }
        return null;
    }

    public static void setupPathPlannerCommands() {
        NamedCommands.registerCommand("Reset QuestNav", new SetQuestNavPoseFromMegaTag1Command());
    }
}
