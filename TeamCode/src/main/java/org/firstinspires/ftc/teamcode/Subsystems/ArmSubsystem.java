package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
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
    public double wristPosition = 0.35;
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

    public void setPower(DcMotor motor, double power) {
        if (pivotMotor != null) {
            motor.setPower(power);
        }
    }

    public void setZeroPowerBehavior(DcMotor motor) {
        if (pivotMotor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }





    public double getCurrentPosition() {
        return pivotMotor != null ? pivotMotor.getCurrentPosition() : 0;
    }

    public double getCurrentPositionWithLimitSwitch() {
        if (limitSwitch != null && !limitSwitch.getState()) {
            pos = getCurrentPosition();
        }
        return getCurrentPosition() - pos;
    }


    public void ArmMovement(Gamepad gamepad1) {

        if (gamepad1.y) {
            pos = DegreesToEncoderTicks(60);
            wristPosition = 0.55;
            elePos = 0;
       } else if (gamepad1.x) {
            pos = DegreesToEncoderTicks(90);
            wristPosition = 0.55;
            elePos = 0;
        } else if (gamepad1.dpad_down) {
            pos = DegreesToEncoderTicks(120);
            wristPosition = 0.35;
            elePos = 0;
        } else if (gamepad1.b) {
            pos = DegreesToEncoderTicks(0);
            wristPosition = 0.2;
            elePos = 0;
        }





        piviotPID.setSetPoint(pos);
        piviotPID.updatePID(getCurrentPositionWithLimitSwitch());
        piviotPID.setMaxOutput(setPowerToPercentage(70));
        piviotPID.setMinOutput(setPowerToPercentage(-70));


        double pidResult = -piviotPID.getResult();
        pivotMotor.setPower(pidResult);

//
//        System.out.println("Target Position: " + pos);
//        System.out.println("Current Position: " + getCurrentPositionWithLimitSwitch());
//        System.out.println("PID Output: " + pidResult);
    }

    public void climb(Gamepad gamepad1) {
        if (!gamepad1.b || !gamepad1.dpad_down || !gamepad1.y || !gamepad1.x) {
            if (gamepad1.a) {
                setPower(pivotMotor,-1.0);
                if (!limitSwitch.getState()) {
                    pos = 0;
                    BRAKE(pivotMotor);
                    setPower(pivotMotor,0.0);
                    System.out.println("Limit switch triggered, position reset.");
                }
            }
        }
    }

    public void manual(Gamepad gamepad1) {
        wristPosition = 0.35;
        if (gamepad1.right_bumper) {
            setPower(pivotMotor, 0.5);

        } else if (gamepad1.left_bumper) {
            setPower(pivotMotor, -0.5);

        } else {
            setZeroPowerBehavior(pivotMotor);
            setPower(pivotMotor, 0);

        }

        if (gamepad1.b) {
            wristPosition = 0.55;
        } else if (gamepad1.a) {
            wristPosition = 0.35;
        }
    }


    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new ArmDefault(this);
    }
}
