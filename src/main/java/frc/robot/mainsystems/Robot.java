// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mainsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain swerve = new Drivetrain();

  // Joysticklere zıbarılmaması için dönüş oranını limitle; 0 dan 1e 1/3 saniyede ulaş
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // X aksisindeki hızı al. Xbox kolları joyistiğ ileri
    // ittiğimizde eksi değeri verdiği için başına eksi koy.
    final var xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
            * Drivetrain.MAX_SPEED;

    // Y aksisindeki hızı yada yanlara/taarruz hızı al. Xbox kolları joyistiğ sağa
    // ittiğimizde artı değeri verdiği için başına eksi koy.
    final var ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * Drivetrain.MAX_SPEED;
    
    // Açısal dönüş hızını al. Joysticki sola tuttuğumuzda 
    // arttı değeri almamız için başına gine eksi koy
    final var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * Drivetrain.MAX_ANGULAR_SPEED;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
