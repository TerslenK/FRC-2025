// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mainsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.subsystems.SwerveModule;

/** ARTIK SWERVE ICIN BIR DRIVETRAINIMIZ VARRR 🤑😍😁 */
public class Drivetrain {
  public static final double MAX_SPEED = 3.0; // TODO Saniyede gideceği maksimum mesafe(metre ile)
  public static final double MAX_ANGULAR_SPEED = Math.PI; // Saniye başı 1/2 tur

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  // TODO Portları degiştir
  private final SwerveModule frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

  // TODO Portu degiştir
  private final AnalogGyro gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
              frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
              kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

  public Drivetrain() {
    gyro.reset();
  }

  /**
   * Joystick bilgisiyle robotu sürme yöntemi
   *
   * @param xSpeed Robotun X aksisi(ön)ndeki hızı.
   * @param ySpeed Robotun Y aksisi(yan)daki hızı.
   * @param rot Robotun açısal oranı.
   * @param fieldRelative Verilen X ve Y aksis değeri alana uygunmu.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Robotun alandaki konumunu güncelle. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
}
