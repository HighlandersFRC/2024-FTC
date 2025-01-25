package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.EncodersTicksToDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.GravityTerm;
import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.BRAKE;
import static org.firstinspires.ftc.teamcode.Tools.Constants.setPowerToPercentage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ArmDefault;

public class ArmSubsystem extends Subsystem {
    public double wristPosition = 0.2;
    public double intakePosition = 0.85;
    private DcMotor pivotMotor;
    private DigitalChannel limitSwitch;

    ArmSubsystem armSubsystem;
    private double pos;
    public double elePos;
    private double manualPower;
    private boolean isManualControlActive = false;
    private boolean armControlToggle = true;

    public ArmSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.pivotMotor = null;
        this.limitSwitch = null;
//        this.pos = 0;
        this.manualPower = 0.0;
        initialize(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap) {
        try {
            pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
            limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");
            // Ensure limitSwitch is set to input mode
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            throw new IllegalStateException("Failed to initialize ArmSubsystem: " + e.getMessage());
        }
    }

    public double getPower() {
        return pivotMotor.getPower();
    }

    public void setPower(double power) {
        if (pivotMotor != null) {
            pivotMotor.setPower(-power);
        }
    }

    public void setZeroPowerBehavior() {
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setPower(0);
    }

    public void HOLDPos() {
        piviotPID.setSetPoint(getCurrentPositionWithLimitSwitch());
        piviotPID.updatePID(getCurrentPositionWithLimitSwitch());
        piviotPID.setMinOutput(-1);
        piviotPID.setMaxOutput(1);
        pivotMotor.setPower(-piviotPID.getResult());
    }


    public double getCurrentPosition() {
        return pivotMotor != null ? pivotMotor.getCurrentPosition() : 0;
    }

    public double getCurrentPositionWithLimitSwitch() {
        if (limitSwitch != null && !limitSwitch.getState()) {
            // Update pos only once when the limit switch is triggered
            if (pos == 0) {
                pos = getCurrentPosition();  // Store initial position when triggered
            }
        }
        return getCurrentPosition() - pos; // Return offset from the initial position
    }


    public double ifLimitSwitchDies(Gamepad gamepad1) {
        boolean buttonPressed = false;

        if (gamepad1.b && !buttonPressed) {
            pos = getCurrentPosition();
            buttonPressed = true;
        } else if (!gamepad1.b) {
            buttonPressed = false;
        }

        return getCurrentPosition() - pos;

    }


    public void ArmMovement(Gamepad gamepad1) {
        if (gamepad1.y) {
            pos = DegreesToEncoderTicks(-60);
            wristPosition = 0;
            intakePosition = 0.85;
            elePos = 2004;
       } else if(gamepad1.b) {
            pos = DegreesToEncoderTicks(0);
            wristPosition = 0;
            intakePosition = 0.85;
            elePos = 0;
        } else if (gamepad1.x) {
            pos = DegreesToEncoderTicks(-20);
            wristPosition = 0;
            intakePosition = 0.85;
            elePos = 1500;
        } else if (gamepad1.a) {
            pos = DegreesToEncoderTicks(-40);
            wristPosition = 0;
            intakePosition = 0.85;
            elePos = 2004;
        }

        piviotPID.setSetPoint(pos);
        piviotPID.updatePID(getCurrentPositionWithLimitSwitch());
        piviotPID.setMaxOutput(setPowerToPercentage(50));
        piviotPID.setMinOutput(setPowerToPercentage(-50));

        pivotMotor.setPower(piviotPID.getResult());


//
//        System.out.println("Target Position: " + pos);
//        System.out.println("Current Position: " + getCurrentPositionWithLimitSwitch());
//        System.out.println("PID Output: " + pidResult);
    }

    public void climb(Gamepad gamepad1) {
        if (!gamepad1.b || !gamepad1.dpad_down || !gamepad1.y || !gamepad1.x) {
            if (gamepad1.a) {
                setPower(-1.0);
                if (!limitSwitch.getState()) {
                    pos = 0;
                    BRAKE(pivotMotor);
                    setPower(0.0);
                    System.out.println("Limit switch triggered, position reset.");
                }
            }
        }
    }



    public void setPosition(double position) {
        piviotPID.setSetPoint(position);
        piviotPID.updatePID(getCurrentPositionWithLimitSwitch());
        piviotPID.setMaxOutput(0.5);
        piviotPID.setMinOutput(-0.5);
        setPower(-piviotPID.getResult());
    }

    public void manual(Gamepad gamepad1) {
        wristPosition = 0.35;
        if (gamepad1.right_bumper) {
            setPower(0.5);
        } else if (gamepad1.left_bumper) {
            setPower(-0.5);
        } else {
            setZeroPowerBehavior();
            setPower(0);
        }

//        if (gamepad1.b) {
//            wristPosition = 0.55;
//        } else if (gamepad1.a) {
//            wristPosition = 0.75;
//        }
//
//
//        if (gamepad1.x) {
//            elePos = 1;
//        } else if (gamepad1.y) {
//            elePos = -1;
//        }
    }


    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new ArmDefault(armSubsystem);
    }
}
