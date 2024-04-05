// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.mainsystems.Drivetrain;

public class SwerveModule {
  private static final double WHEEL_RADIUS = 0.0508; // Tekerleğin inç değeri (1/16)
  private static final int ENCODER_RESOLUTION = 4096;

  private static final double MODULE_MAX_ANGULAR_VELOCITY = Drivetrain.MAX_ANGULAR_SPEED;
  private static final double MODULE_MAX_ANGULAR_ACCELERATION =
      2 * Math.PI; // RPP(Saniye Başı Radyan) üssü 2

  private final PWMSparkMax driveMotor;
  private final PWMSparkMax turningMotor;

  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  // TODO Robota göre modifiye et
  private final PIDController drivePIDController = new PIDController(1, 0, 0);

  // TODO Robota göre modifiye et
  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
                  MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

  // TODO Robota göre modifiye et
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Hareket motoru, dönme motoru ve bunların encoderi ile yeni bir swerve modülü oluştur.
   *
   * @param driveMotorChannel Sürüş motorunun PWM portu.
   * @param turningMotorChannel Dönme motorunun PWM portu.
   * @param driveEncoderChannelA "A" Kanalındaki sürüş encoderi için DIO girişi.
   * @param driveEncoderChannelB "B" Kanalındaki sürüş encoderi için DIO girişi.
   * @param turningEncoderChannelA "A" Kanalındaki dönüş encoderi için DIO girişi.
   * @param turningEncoderChannelB "B" Kanalındaki dönüş encoderi için DIO girişi.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    driveMotor = new PWMSparkMax(driveMotorChannel);
    turningMotor = new PWMSparkMax(turningMotorChannel);

    driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Sürüş encoderinin DPS'ini ayarla.
    // 1 tekerlek dönüşünü encoderin çözünürlüğüne
    // bölerek elde ediyoruz.
    driveEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

    // Dönüş encoderi için RPP(Sinyal Başı Radyan) ile mesafe(açı) çek.
    // Bunuda tam dönme(2*pi)yi encoder çözünürlüğüne bölerek bul.
    turningEncoder.setDistancePerPulse(2 * Math.PI / ENCODER_RESOLUTION);

    // PID Kontrolcüsünün değerini "-pi" ile "pi" arasında sınırlandırıp sürekli olmasını sağla
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Modülün şuanki durumunu söyle.
   *
   * @return modülün şuanki durumu.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Modülün şuanki konumunu söyle.
   *
   * @return modülün şuanki konumu.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Modülün ne duruma gelmesini söyle.
   *
   * @param desiredState istenilen hız ve rotasyonu olan durum.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(turningEncoder.getDistance());

    // İstenilen durumun motoru 90 dereceden fazla döndürmesini engelle
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Daha kaygan😏 bir sürüş için hızı hata payının kosinüsü olarak al.
    // Bu sayede vektörü çapraz almak yerine düz bi şekilde elde et
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Sürüş PID Kontrolcüsündeki çıktıyı hesapla
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

    // Dönüş PID Kontrolcüsündeki çıktıyı hesapla
    final double turnOutput =
        turningPIDController.calculate(turningEncoder.getDistance(), state.angle.getRadians());

    final double turnFeedforward =
        this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
