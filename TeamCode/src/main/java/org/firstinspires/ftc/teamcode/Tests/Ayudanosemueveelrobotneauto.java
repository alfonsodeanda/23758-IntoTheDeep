package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "PRUEBA ENERO")
public class Ayudanosemueveelrobotneauto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe;
    private DcMotor Intake = null;
    private DcMotor ElevadorIz = null;
    private DcMotor ElevadorDe = null;
    private DcMotor Outtake = null;
    private Servo ServoOuttake = null;
    private Servo Muñeca = null;
    private CRServo Brazo = null;

    //variables para el Toggle en las garras de intake y outtake
    private boolean outtakeMode = false; //para saber el modo en el que esta el outtake (abrir/cerrar)
    private boolean outtakeButtonState = false; //para saber si el boton ha sido presionado
    private boolean intakeMode = false; //para saber el modo en el que esta el intake(abrir/cerrar)
    private boolean intakeButtonState = false; //para saber si el boton ha sido presionado
    private boolean transferButtonState = false; //para saber si el boton ha sido presionado


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

        // Direcciones
        DelanteIz.setDirection(DcMotor.Direction.FORWARD);
        DelanteDe.setDirection(DcMotor.Direction.REVERSE);
        AtrasIz.setDirection(DcMotor.Direction.FORWARD);
        AtrasDe.setDirection(DcMotor.Direction.REVERSE);
        Muñeca.setDirection(Servo.Direction.REVERSE);

        // Comportamiento al soltar
        zeroPowerBehaviorBrake();

        resetEncoders();

        waitForStart();
        runtime.reset();

        //Programa principal, se repite por siempre
        if (opModeIsActive()) {
            DelanteDe.setPower(1);
            sleep(1000);

            //imprimir ciertos valores para debuggear
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Riel", Intake.getCurrentPosition());
            telemetry.addData("Muñeca outtake:", Outtake.getCurrentPosition());
            telemetry.update();
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
    public void resetEncoders() {
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
