//package org.firstinspires.ftc.teamcode.Commands;
//
//import static org.firstinspires.ftc.teamcode.Tools.Robot.elevators;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
//import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Tools.Robot;
//
//public class ElevatorWithPower implements Command {
//    private double power;
//
//    public ElevatorWithPower() {
//    }
//
//    @Override
//    public void start() {
//    }
//
//    @Override
//    public void execute() {
//        power = Robot.elevatorPower;
//        if (power > 0) {
//            elevators.moveLeftElevator(1);
//            elevators.moveRightElevator(1);
//        } else if (power < 0) {
//            elevators.moveLeftElevator(-1);
//            elevators.moveRightElevator(-1);
//        } else {
//            Elevators.stop();
//        }
//    }
//
//    @Override
//    public void end() {
//        Elevators.stop();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return /*(power == 0)*/false;
//    }
//
//    @Override
//    public Subsystem getRequiredSubsystem() {
//        return elevators;
//    }
//}
