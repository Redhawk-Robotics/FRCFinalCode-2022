// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleAuto extends CommandBase {

 // private   Robot m_robot;

  private  CANSparkMax outtake;
  private  CANSparkMax outtake2;
  private  CANSparkMax uptake;

  private MotorControllerGroup leftSpeedGroup;
  private MotorControllerGroup rightSpeedGroup;

  private DifferentialDrive robotDrive;

  private double startTime;


  /** Creates a new SimpleAuto. */
  public SimpleAuto( double startTimelocal, CANSparkMax outtake,  CANSparkMax outtake2, CANSparkMax uptake, 
    MotorControllerGroup leftSpeedGroup, MotorControllerGroup rightSpeedGroup, DifferentialDrive robotDrive) {

    this.outtake = outtake;
    this.outtake2 = outtake2;
    this.uptake = uptake;

    this.leftSpeedGroup = leftSpeedGroup;
    this.rightSpeedGroup = rightSpeedGroup;

    this.robotDrive = robotDrive;


    this.startTime = startTimelocal;

  }
    


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double time = Timer.getFPGATimestamp();

    if (time - startTime < 3) {

      this.outtake.set(.125);;
      this.outtake2.set(.5);

      this.uptake.set(1);

    } else if (time - startTime > 3 && time - startTime < 8) {
        this.leftSpeedGroup.set(.2);
        this.rightSpeedGroup.set(-.2);
      } else {
      this.robotDrive.arcadeDrive(0, 0);

      this.outtake2.set(0);
      this.outtake.set(0);

      this. uptake.set(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
