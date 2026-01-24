// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ShooterCalculator {

    private ShooterCalculator() {
    }

    /**
     * Computes the field-relative bearing from the robot pose to a field target.
     *
     * @param robotPose current robot pose on the field
     * @param targetPos target position on the field
     * @return Rotation2d representing the angle (in field coordinates) from robot
     *         to target
     */
    public static Rotation2d fieldBearingToTarget(Pose2d robotPose, Translation2d targetPos) {
        double dx = targetPos.getX() - robotPose.getX();
        double dy = targetPos.getY() - robotPose.getY();

        // atan2(y, x) -> angle from +X axis, CCW positive (matches WPILib's convention)
        double angleRad = Math.atan2(dy, dx);
        return new Rotation2d(angleRad);
    }

    /**
     * Computes the turret-relative angle needed to aim at a field target,
     * assuming turret 0° == robot 0°.
     *
     * @param robotPose current robot pose on the field
     * @param targetPos target position on the field
     * @return Rotation2d turret angle (robot-relative) to point at the target
     */
    public static Rotation2d turretAngleForFieldTarget(Pose2d robotPose, Translation2d targetPos) {
        Rotation2d fieldBearing = fieldBearingToTarget(robotPose, targetPos);
        Rotation2d robotHeading = robotPose.getRotation();

        // Turret-relative = fieldBearing - robotHeading
        Rotation2d turretAngle = fieldBearing.minus(robotHeading);

        // Optional: normalize to -pi..pi if you care about shortest-path motion
        return new Rotation2d(Math.IEEEremainder(turretAngle.getRadians(), 2.0 * Math.PI));
    }

    public static Rotation2d turretAngleForFieldTarget(Pose2d robotPose, Pose2d targetPose) {
        return turretAngleForFieldTarget(robotPose, targetPose.getTranslation());
    }

    
}
