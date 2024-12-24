package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ArmDefault;

public class ArmSubsystem extends Subsystem {
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch;

    protected static double pos = 0;

    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");
    }

    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }
    public static double getCurrentPosition() {
        return pivotMotor.getCurrentPosition();
    }
    public static double getCurrentPositionWithLimitSwitch() {

        if (!limitSwitch.getState()) {
            pos = ArmSubsystem.getCurrentPosition();
        }
        return ArmSubsystem.getCurrentPosition() - pos;
    }
    public ArmSubsystem(String name) {
        super(name);
    }
    public static void ArmMovement(Gamepad gamepad1) {
      if(gamepad1.left_bumper) {
          ArmSubsystem.setPower(1);
      } else if(gamepad1.right_bumper) {
          ArmSubsystem.setPower(-1);
      }

    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new ArmDefault());
    }

    @Override
    public Command getDefaultCommand() {
        return new ArmDefault();
    }


}