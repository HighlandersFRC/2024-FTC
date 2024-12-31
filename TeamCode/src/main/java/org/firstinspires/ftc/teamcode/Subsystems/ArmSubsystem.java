package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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
import org.firstinspires.ftc.teamcode.Tools.Constants;

public class ArmSubsystem extends Subsystem {
    public double wristPosition = 0.35;
    private DcMotor pivotMotor;
    private DigitalChannel limitSwitch;
    ArmSubsystem armSubsystem;
    private double pos;
    private double manualPower;
    private boolean isManualControlActive = false;
    private boolean armControlToggle = true;

    public ArmSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.pivotMotor = null;
        this.limitSwitch = null;
        this.pos = 0;
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

    public void setPower(double power) {
        if (pivotMotor != null) {
            pivotMotor.setPower(power);
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
            pos = DegreesToEncoderTicks(-45);
            wristPosition = 0.55;
        } else if (gamepad1.x) {
            pos = DegreesToEncoderTicks(90);
            wristPosition = 0.55;
        } else if (gamepad1.dpad_down) {
            pos = DegreesToEncoderTicks(120);
            wristPosition = 0.35;
        } else if (gamepad1.b) {
            pos = DegreesToEncoderTicks(0);
            wristPosition = 0.2;
        }


        piviotPID.setSetPoint(pos);
        piviotPID.updatePID(getCurrentPositionWithLimitSwitch());
        piviotPID.setMaxOutput(setPowerToPercentage(70));
        piviotPID.setMinOutput(setPowerToPercentage(-70));

        double pidResult = -piviotPID.getResult();
        pivotMotor.setPower(pidResult);


        System.out.println("Target Position: " + pos);
        System.out.println("Current Position: " + getCurrentPositionWithLimitSwitch());
        System.out.println("PID Output: " + pidResult);
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

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new ArmDefault(this);
    }
}
