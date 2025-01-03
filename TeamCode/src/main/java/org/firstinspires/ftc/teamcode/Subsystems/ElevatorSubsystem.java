package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends Subsystem  {
    private DcMotor Elevator;
    ArmSubsystem armSubsystem;
    public ElevatorSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.Elevator = null;
        initialize(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap) {
        Elevator = hardwareMap.dcMotor.get( "Elevator");
    }




    public void manual(Gamepad gamepad1) {
        double currentPosition = Elevator.getCurrentPosition();


        if (gamepad1.right_bumper && currentPosition < MAX_TICKS) {
            Elevator.setPower(0.5);
        } else if (gamepad1.left_bumper && currentPosition > MIN_TICKS) {
            Elevator.setPower(-0.5);
        } else {
            Elevator.setPower(0);
        }


    }


    public void contolElevatorSetPoint(Gamepad gamepad1) {
        armSubsystem.elePos = Math.max(MIN_TICKS, Math.min(MAX_TICKS, armSubsystem.elePos));

        elevatorPID.setSetPoint(armSubsystem.elePos);

        elevatorPID.updatePID(Elevator.getCurrentPosition());
        elevatorPID.setMaxOutput(0.5);
        elevatorPID.setMinOutput(-0.5);
        Elevator.setPower(elevatorPID.getResult());
    }



    public void contolElevator(Gamepad gamepad1) {
        double currentPosition = Elevator.getCurrentPosition();

        if (gamepad1.right_bumper && currentPosition < MAX_TICKS) {
            Elevator.setPower(0.5);
        } else if (gamepad1.left_bumper && currentPosition > MIN_TICKS) {
            Elevator.setPower(-0.5);
        } else {
            Elevator.setPower(0);
        }

        armSubsystem.elePos = Math.max(MIN_TICKS, Math.min(MAX_TICKS, armSubsystem.elePos));

        elevatorPID.setSetPoint(armSubsystem.elePos);

        elevatorPID.updatePID(Elevator.getCurrentPosition());
        elevatorPID.setMaxOutput(0.5);
        elevatorPID.setMinOutput(-0.5);
        Elevator.setPower(elevatorPID.getResult());
    }

}
