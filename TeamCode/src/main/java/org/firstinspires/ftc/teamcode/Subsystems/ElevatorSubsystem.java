package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends Subsystem {
    public DcMotor Elevator;
    private double elePos = 100;

    public ElevatorSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.Elevator = null;
        initialize(hardwareMap);
    }

    public double getCurrentPosition(DcMotor motor) {
        return motor.getCurrentPosition();
    }


        public void initialize(HardwareMap hardwareMap) {
        Elevator = hardwareMap.dcMotor.get("Elevator");
    }



    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }


    public void manual(Gamepad gamepad1) {


        if (gamepad1.right_bumper) {
            Elevator.setPower(1);
        } else if (gamepad1.left_bumper) {
            Elevator.setPower(-1);
        } else {
            Elevator.setPower(0);
        }


    }

    public void setPosition(DcMotor motor, double pos) {
        elevatorPID.setSetPoint(pos);
        elevatorPID.updatePID(motor.getCurrentPosition());
        elevatorPID.setMaxOutput(1);
        elevatorPID.setMinOutput(-1);
        motor.setPower(-elevatorPID.getResult());
    }


    public void contolElevatorSetPoint(Gamepad gamepad1) {
        if (!gamepad1.right_bumper || !gamepad1.left_bumper) {
            if (gamepad1.a) {
                elePos = -5000;
            } else if (gamepad1.b) {
                elePos = -100;
            } else if (gamepad1.x) {
                elePos = -3000;
            }
        } else {
            if (gamepad1.right_bumper) {
                Elevator.setPower(1);
            } else if (gamepad1.left_bumper) {
                Elevator.setPower(-1);
            } else {
                Elevator.setPower(0);
            }
        }


        elevatorPID.setSetPoint(elePos);
        elevatorPID.updatePID(Elevator.getCurrentPosition());
        elevatorPID.setMaxOutput(1);
        elevatorPID.setMinOutput(-1);
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

        if (!gamepad1.right_bumper || !gamepad1.left_bumper) {
            if (gamepad1.a) {
                elePos = -5000;
            } else if (gamepad1.b) {
                elePos = -100;
            } else if (gamepad1.x) {
                elePos = -3000;
            }

            elevatorPID.setSetPoint(elePos);

            elevatorPID.updatePID(Elevator.getCurrentPosition());
            elevatorPID.setMaxOutput(0.5);
            elevatorPID.setMinOutput(-0.5);
            Elevator.setPower(elevatorPID.getResult());
        }

    }
}
