package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AUTO TEMPLATE FINAL TOLUCA")
public class Ayudanosemueveelrobotneauto extends LinearOpMode {
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe;
    private DcMotor Intake = null;
    private DcMotor ElevadorIz = null;
    private DcMotor ElevadorDe = null;
    private DcMotor Outtake = null;
    private Servo ServoOuttake = null;
    private Servo Muñeca = null;
    private CRServo Brazo = null;
    public IMU imu;

    final double outtake_open = 0.5;
    final double outtake_close = 0.2;

    final double intake_open = 0.4;
    final double intake_close = 0.55;

    private static final int maxEncoderRiel = -1700; //constante de el numero maximo de ticks que se puede recorrer el riel
    private static final int maxOuttake = -150;
    private static final double cmByTick = 0.053855874; //constante que define cuantos cm hay en cada tick de los motores el chasis


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motores chasis
        DelanteIz = hardwareMap.get(DcMotor.class, "DelanteIz");
        DelanteDe = hardwareMap.get(DcMotor.class, "DelanteDe");
        AtrasIz = hardwareMap.get(DcMotor.class, "AtrasIz");
        AtrasDe = hardwareMap.get(DcMotor.class, "AtrasDe");
        //Motores mecanismos
        ElevadorIz = hardwareMap.get(DcMotor.class, "elevadorIz"); //Uno de los dos motores del elevador
        ElevadorDe = hardwareMap.get(DcMotor.class, "elevadorDe");
        Intake = hardwareMap.get(DcMotor.class, "Intake"); //Motor del riel
        Outtake = hardwareMap.get(DcMotor.class, "outtake"); //Motor de la muñeca del outtake
        //Servos mecanismos
        ServoOuttake = hardwareMap.get(Servo.class, "garraO"); //servo de la garra del outtake
        Muñeca = hardwareMap.get(Servo.class, "muñeca"); //servo de la garra del intake
        Brazo = hardwareMap.get(CRServo.class, "brazo"); //servo de la muneca del intake

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                        )
                )
        );

        // Direcciones
        DelanteIz.setDirection(DcMotor.Direction.FORWARD);
        DelanteDe.setDirection(DcMotor.Direction.REVERSE);
        AtrasIz.setDirection(DcMotor.Direction.FORWARD);
        AtrasDe.setDirection(DcMotor.Direction.REVERSE);
        Muñeca.setDirection(Servo.Direction.REVERSE);

        // Comportamiento al soltar
        zeroPowerBehaviorBrake();

        resetChassisEncoders();
        resetMechEncoders();

        imu.resetYaw();

        waitForStart();

        //Programa principal, se repite por siempre
        if (opModeIsActive()) {
            outtake(outtake_close);
            intake(intake_close);

            goForward(60.96, 0.3);
            turnToAngle(45, 0.5);
            goForward(20, 0.3);
            elevador(-1000, 1);
            outtakeWrist(-140, 0.6);
            outtake(outtake_open);
        }
    }

    //settea todos los motores en brake cuando no se les aplica potencia
    public void zeroPowerBehaviorBrake(){
        DelanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DelanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopMotors() {
        motorsSetPower(0, 0, 0, 0);
    }

    //se configuran los encoders para poder acceder a las lecturas y configurar limites
    public void resetChassisEncoders() {
        DelanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DelanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AtrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AtrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DelanteIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DelanteDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AtrasIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AtrasDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMechEncoders() {
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getAngle () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

    public void goForward (double cm, double maxPower) {
        resetChassisEncoders();
        int targetPos = (int) (cm/cmByTick);

        double kP = 0.05;
        double kI = 0;
        double kD = 0;

        while (opModeIsActive()){
            int currentPos = encoderAverage();

            double error = targetPos - currentPos;

            double power = (kP * error);
            power = Math.max(-maxPower, Math.min(power, maxPower));

            motorsSetPower(power, power, power, power);

            if(Math.abs(error) <=1) break;

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

    public void turnToAngle (double targetAngle, double maxPower) {
        double kP = 0.1;

        while (opModeIsActive()) {
            double currentAngle = getAngle();
            double error = targetAngle - currentAngle;
            double power = (kP * error);
            power = Math.max(-maxPower, Math.min(power, maxPower));

            motorsSetPower(-power, power, -power, power);

            if(Math.abs(error) <= 0.01) break;

            telemetry.addData("Angle: ", getAngle());
            telemetry.addData("Error: ", error);
            telemetry.addData("Power: ", power);
            telemetry.update();
        }
        stopMotors();
    }

    public void elevador(int pos, double power) {
        ElevadorIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorIz.setTargetPosition(pos);
        ElevadorDe.setTargetPosition(pos);
        ElevadorIz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevadorDe.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevadorIz.setPower(power);
        ElevadorDe.setPower(power);

        while (opModeIsActive() && (ElevadorIz.isBusy() || ElevadorDe.isBusy()) ) {
            telemetry.addData("Elevador", "Posición: %d", ElevadorIz.getCurrentPosition());
        }
        telemetry.update();
        ElevadorIz.setPower(0);
        ElevadorDe.setPower(0);
    }

    public void riel(int pos, double power) {
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setTargetPosition(pos);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(power);

        while (opModeIsActive() && Intake.isBusy()) {
            telemetry.addData("Riel", "Posición: %d", Intake.getCurrentPosition());
        }

        Intake.setPower(0);
    }

    public void outtakeWrist(int pos, double power) {
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setTargetPosition(pos);
        Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake.setPower(power);

        while (opModeIsActive() && Outtake.isBusy()) {
            telemetry.addData("Muñeca Outtake", "Posición: %d", Outtake.getCurrentPosition());
        }

        Outtake.setPower(0);
    }

    public void intake(double pos){
        Muñeca.setPosition(pos);
    }

    public void outtake(double pos){
        ServoOuttake.setPosition(pos);
    }

    public void intakeWrist(long time, double pow){
        Brazo.setPower(pow);
        sleep(time);
        Brazo.setPower(0);
    }

}
