//package org.firstinspires.ftc.teamcode.Commands;
//
//import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//
//public class Gamepad1Climb implements Command{
//    public double Arm_power;
//    public boolean STOP;
//    String name = "Arm";
//    ArmSubsystem Arm;
//    public Gamepad1Climb(ArmSubsystem arm, double targetPos) {Arm=arm ;}
//    @Override
//    public void start() {
//        STOP = false;
//    }
//public Gamepad1Climb(double power) {
//        power = Arm_power;
//    STOP = false;
//}
//
//    @Override
//    public void execute() {
//        ArmSubsystem.setPower(Arm_power);
//        STOP = false;
//    }
//
//    @Override
//    public void end() {
//        STOP = true;
//    }
//
//    @Override
//    public boolean isFinished() {
//        return STOP;
//    }
//
//    @Override
//    public Subsystem getRequiredSubsystem() {
//        return Arm;
//    }
//}
