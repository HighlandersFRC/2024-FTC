package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.IntakeDefault;
public class IntakeSubsystem extends Subsystem {
   public Servo RightIntake;
    public Servo LeftIntake;

    public double intakePosRight = 0.7;
    public double intakePosLeft = 0;

    public IntakeSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.RightIntake = null;
        this.LeftIntake = null;
        initialize(hardwareMap);
    }
    public double getPositionRight() {
        return RightIntake.getPosition();
    }

    public double getPositionLeft() {
        return LeftIntake.getPosition();
    }

    public void initialize(HardwareMap hardwareMap) {
        RightIntake = hardwareMap.servo.get("IntakeRight");
        LeftIntake = hardwareMap.servo.get("IntakeLeft");
    }




    public void controlIntake(Gamepad gamepad1) {
        intakePosRight = 0.7;
        intakePosLeft = 0.1;
        if (gamepad1.left_trigger != 0) {
            intakePosRight = 0.1;
            intakePosLeft = 0.7;
        }
        RightIntake.setPosition(intakePosRight);
        LeftIntake.setPosition(intakePosLeft);
        }





    public void setPosition(double RightPos, double LeftPos) {
        RightIntake.setPosition(RightPos);
        RightIntake.setPosition(LeftPos);
    }


    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new IntakeDefault());
    }

    @Override
    public Command getDefaultCommand() {
        return new IntakeDefault();
    }
}
