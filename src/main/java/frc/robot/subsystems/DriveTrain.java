// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


/**import static frc.robot.utility.Constants.Unit.*; */

public class DriveTrain extends SubsystemBase {
  private static final int LEFT_FRONT_MOTOR_PORT = Constants.LEFT_FRONT_MOTOR_PORT;
  private static final int LEFT_BACK_MOTOR_PORT = Constants.LEFT_BACK_MOTOR_PORT;
  private static final int RIGHT_FRONT_MOTOR_PORT = Constants.RIGHT_BACK_MOTOR_PORT;
  private static final int RIGHT_BACK_MOTOR_PORT = Constants.RIGHT_BACK_MOTOR_PORT;

  private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = Constants.LEFT_FRONT_ENCODER_ROTATOR_PORT;
  private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = Constants.LEFT_BACK_ENCODER_ROTATOR_PORT;
  private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = Constants.RIGHT_FRONT_ENCODER_ROTATOR_PORT;
  private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = Constants.RIGHT_BACK_ENCODER_ROTATOR_PORT;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}
  
  private final static TalonFX[] motors = new TalonFX[] {
    new TalonFX(LEFT_FRONT_MOTOR_PORT),
    new TalonFX(LEFT_BACK_MOTOR_PORT),
    new TalonFX(RIGHT_FRONT_MOTOR_PORT),
    new TalonFX(RIGHT_BACK_MOTOR_PORT)
  };
  

  private final CANcoder[] encoders = new CANcoder[] {
    new CANcoder(LEFT_FRONT_ENCODER_ROTATOR_PORT),
    new CANcoder(LEFT_BACK_ENCODER_ROTATOR_PORT),
    new CANcoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT),
    new CANcoder(RIGHT_BACK_ENCODER_ROTATOR_PORT)
  };

  DifferentialDrive drive = new DifferentialDrive(motors[0], motors[2]);
  GroupMotorControllers
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double spd) {
    motors[0].set(spd);
    motors[1].set(spd);
    motors[2].set(spd);
    motors[3].set(spd);
  }


  public double getMotorPosition(int motornum){
    return motors[motornum].getSelectedSensorPosition(0);
  }

  public void driveWithJoysticks(XboxController controller, double speed){
    drive.arcadeDrive(controller.getRawAxis(Constants.XBOX_LEFT_Y_AXIS)*speed, controller.getRawAxis(Constants.XBOX_Left_X_Axis)*speed);
  }
}
