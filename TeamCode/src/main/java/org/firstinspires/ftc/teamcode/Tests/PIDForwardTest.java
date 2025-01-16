package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "PID Adelante")
public class PIDForwardTest extends LinearOpMode {
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe; //motores

    private static final double cmByTick = 0.053855874; //constante que define cuantos cm hay en cada tick de los motores el chasis

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Prueba de control PID para ir adelante de forma precisa");
        telemetry.update();

        DelanteIz = hardwareMap.get(DcMotor.class, "DelanteIz");
        DelanteDe = hardwareMap.get(DcMotor.class, "DelanteDe");
        AtrasIz  = hardwareMap.get(DcMotor.class, "AtrasIz");
        AtrasDe = hardwareMap.get(DcMotor.class, "AtrasDe");

        DelanteIz.setDirection(DcMotor.Direction.FORWARD);
        DelanteDe.setDirection(DcMotor.Direction.REVERSE);
        AtrasIz.setDirection(DcMotor.Direction.FORWARD);
        AtrasDe.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        zeroPowerBehaviorBrake();

        waitForStart();

        if (opModeIsActive()){
            PIDForward(60.96, 0.4);
        }
    }

    public void stopMotors() {
        motorsSetPower(0, 0, 0, 0);
    }

    public void resetEncoders() {
        DelanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DelanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AtrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AtrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DelanteIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DelanteDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AtrasIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AtrasDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void zeroPowerBehaviorBrake() {
        DelanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DelanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        DelanteIz.setPower(powDeIz);
        DelanteDe.setPower(powDeDe);
        AtrasIz.setPower(powAtIz);
        AtrasDe.setPower(powAtDe);
    }

    public int encoderAverage () {
        return (DelanteIz.getCurrentPosition() + DelanteDe.getCurrentPosition() +
                AtrasIz.getCurrentPosition() + AtrasDe.getCurrentPosition()) / 4;
    }

    public void PIDForward (double cm, double maxPower) {
        resetEncoders();
        int targetPos = (int) (cm/cmByTick);

        //double integral = 0;
        //double lastError = 0;

        double kP = 0.02;
        double kI = 0;
        double kD = 0;

        while (opModeIsActive()){
            int currentPos = encoderAverage();

            double error = targetPos - currentPos;
            //integral += error;
            //double derivative = error - lastError;

            double power = (kP * error); // + (kI * integral) + (kD * derivative);
            power = Math.max(-maxPower, Math.min(power, maxPower));

            motorsSetPower(power, power, power, power);

            //lastError = error;

            //if(targetPos==currentPos) break;

            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    DelanteDe.getCurrentPosition(), DelanteIz.getCurrentPosition(),
                    AtrasDe.getCurrentPosition(), AtrasIz.getCurrentPosition());
            telemetry.addData("Average encoder value: ", encoderAverage());
            telemetry.addData("Target position: ", targetPos);
            telemetry.addData("Error: ", error);
            telemetry.addData("Power: ", power);
            telemetry.update();
        }
        stopMotors();
    }
}
