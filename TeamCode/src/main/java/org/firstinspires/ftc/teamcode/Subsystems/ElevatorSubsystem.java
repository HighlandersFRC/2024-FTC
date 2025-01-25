package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.setPowerToPercentage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends Subsystem {
    public DcMotor Elevator;
    private double elePos = -500;

    double power = 0;

    public ElevatorSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.Elevator = null;
        initialize(hardwareMap);
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double getCurrentPosition() {
        return Elevator.getCurrentPosition();
    }


        public void initialize(HardwareMap hardwareMap) {
        Elevator = hardwareMap.dcMotor.get("Elevator");
    }



    public void setPower(double power) {
        Elevator.setPower(power);
    }

    public void manual(Gamepad gamepad1) {
//
//        double currentPosition = getCurrentPosition();
//        System.out.println("asxsssdf");
//        System.out.println(gamepad1.b &&!(currentPosition < -300));
//
//            if (!(currentPosition > MIN_TICKS - 100)&&gamepad1.a) {
//                System.out.println("First Passed");
//
//                    System.out.println("Second Passed");
//                    power = setPowerToPercentage(100);
//                    setPower(setPowerToPercentage(100));
//
//            } else if (gamepad1.b && !(currentPosition < -300)) {
//                System.out.println("b");
//                power = setPowerToPercentage(-100);
//                setPower(setPowerToPercentage(-100));
//
//            } else {
//                System.out.println("else Statement");
//                Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                setPower(0);
//            }

        if (gamepad1.left_bumper){
            setPower(0.8);
        } else if (gamepad1.right_bumper) {
            setPower(-0.8);
        } else {
            Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setPower(0);
        }

    }

    public void setPosition(double pos) {
        elevatorPID.setSetPoint(pos);
        elevatorPID.updatePID(Elevator.getCurrentPosition());
        elevatorPID.setMaxOutput(1);
        elevatorPID.setMinOutput(-1);
        Elevator.setPower(elevatorPID.getResult());
    }


    public void contolElevatorSetPoint(Gamepad gamepad1) {
            if (gamepad1.a) {
                elePos = -2000;
            } else if (gamepad1.b) {
                elePos = 0;
            } else if (gamepad1.x) {
                elePos = -1600;
            }

        elevatorPID.setSetPoint(elePos);
        elevatorPID.updatePID(Elevator.getCurrentPosition());
        elevatorPID.setMaxOutput(0.8);
        elevatorPID.setMinOutput(-0.8);
        Elevator.setPower(elevatorPID.getResult());
    }


    public void contolElevator(Gamepad gamepad1) {
        double currentPosition = getCurrentPosition();

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
