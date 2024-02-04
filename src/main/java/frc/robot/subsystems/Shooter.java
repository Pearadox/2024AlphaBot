// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.util.PearadoxSparkMax;
import frc.robot.Constants.ShootConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private PearadoxSparkMax shootMotor;
  private PearadoxSparkMax shootPitchMotor;
  private PearadoxSparkMax shootYawMotor;

  private RelativeEncoder pitchEncoder;


  private static final Shooter shooter = new Shooter();

  public static Shooter getInstance(){
    return shooter;
  }

  public Shooter() {
    shootMotor = new PearadoxSparkMax(ShootConstants.SHOOT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast, 50, false);
    shootPitchMotor = new PearadoxSparkMax(ShootConstants.SHOOT_PITCH_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
    shootYawMotor = new PearadoxSparkMax(ShootConstants.SHOOT_YAW_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast, 50, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void shoot() {

    shootMotor.set(ShootConstants.SHOOTING_SPEED);
  }
  private void stopShoot()
  {
    shootMotor.set(0);
  }
}
