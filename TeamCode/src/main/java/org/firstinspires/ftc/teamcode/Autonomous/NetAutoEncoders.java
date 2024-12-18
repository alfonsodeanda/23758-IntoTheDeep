package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AutoIzqEncoders15pts2")
public class NetAutoEncoders extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz;
    private DcMotor delanteDe;
    private DcMotor atrasIz;
    private DcMotor atrasDe;
    private DcMotor Intake;
    private DcMotor elevador;
    private CRServo garra;

    private boolean garraAbierta = false;

    private static final double pi = 3.14159265;
    private static final int CPR = 560;
    private static final int wheelDiameter = 96;
    private static final double wheelCircumference = 30.15928944;
    private static final double cmByTick = 0.053855874;
    private static final double foamTile = 60.96;
    private static final int subirElevadorHi = 11800;
    private static final int bajarElevadorHi = -11875;
    private static final int subirElevadorLow = 6000;
    private static final int bajarElevadorLow = -6075;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Autonomo por encoders. Inicia en el tapete A5. Anota precargado. Anota sample. Se estaciona en Obs Zone.");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz  = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");
        elevador  = hardwareMap.get(DcMotor.class, "elevador");
        garra = hardwareMap.get(CRServo.class,"garra");
        Intake = hardwareMap.get(DcMotor.class,"Intake");

        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        delanteDe.setDirection(DcMotor.Direction.FORWARD);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);

        elevador.setDirection(DcMotor.Direction.REVERSE);
        garra.setDirection(CRServo.Direction.REVERSE);

        delanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        delanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()){
            //AUTONOMO PROGRAMA PRINCIPAL
            garraMovimiento(1);
            goForward(70, 1); //avanza un tapete

            turning(-1300, 1300, 0.8); //-135grados desde el origen aprox.

            actionsToBasket(subirElevadorHi, bajarElevadorHi);

            goForward(20, 0.8);
            turning(500,-500, 0.8); //-90grados desde el origen

            strafe(1, 65, 0.8);
            Intake.setPower(-1);
            goForward(30, 0.8);
            Intake.setPower(0);
            goForward(-30, -0.8);

            strafe(0, 60, 0.8); //fin vid
            turning(-500,500, 0.8);

            actionsToBasket(subirElevadorLow, bajarElevadorLow);

            turning(1300, -1300, 0.8);
            goForward(-50, -1);
            strafe(200, 65, 0.8);

        }
    }

    public void stopMotors() {
        delanteIz.setPower(0);
        delanteDe.setPower(0);
        atrasIz.setPower(0);
        atrasDe.setPower(0);
    }

    public void motorsSetModeSTOPANDRESET () {
        delanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void motorsSetModeRUNTOPOSITION () {
        delanteIz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        delanteDe.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        atrasIz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        atrasDe.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void motorsSetTargetPosition (int posDeIz, int posDeDe, int posAtIz, int posAtDe) {
        delanteIz.setTargetPosition(posDeIz);
        delanteDe.setTargetPosition(posDeDe);
        atrasIz.setTargetPosition(posAtIz);
        atrasDe.setTargetPosition(posAtDe);
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        delanteIz.setPower(powDeIz);
        delanteDe.setPower(powDeDe);
        atrasIz.setPower(powAtIz);
        atrasDe.setPower(powAtDe);
    }

    public void goForward(double cm, double power) {
        int pos = (int) (cm/cmByTick);

        motorsSetModeSTOPANDRESET();

        motorsSetTargetPosition(pos, pos, pos, pos);

        motorsSetModeRUNTOPOSITION();

        for (double i=0.1; i<=power; i+=0.05){
            motorsSetPower(i, i, i, i);
            sleep(50);
        }

        while (opModeIsActive() && (delanteIz.isBusy() || delanteDe.isBusy() || atrasIz.isBusy() || atrasDe.isBusy())) {
            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    delanteDe.getCurrentPosition(), delanteIz.getCurrentPosition(),
                    atrasDe.getCurrentPosition(), atrasIz.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void strafe(int leftRight, double cm, double power) {
        double nPower=power*-1;
        int pos = (int) (cm/cmByTick);

        motorsSetModeSTOPANDRESET();

        if(leftRight==1){ //si el primer parametro de la funcion es 1, strafear a la derecha
            motorsSetTargetPosition(delanteIz.getCurrentPosition() + pos,
                    delanteDe.getCurrentPosition() - pos,
                    atrasIz.getCurrentPosition() - pos,
                    atrasDe.getCurrentPosition() + pos);
        } else{ //si es 0, strafear a la izquierda
            motorsSetTargetPosition(delanteIz.getCurrentPosition() - pos,
                    delanteDe.getCurrentPosition() + pos,
                    atrasIz.getCurrentPosition() + pos,
                    atrasDe.getCurrentPosition() - pos);
        }

        motorsSetModeRUNTOPOSITION();
        sleep(50);

        motorsSetPower(power, nPower, nPower, power);

        while (opModeIsActive() && (delanteIz.isBusy() || delanteDe.isBusy() || atrasIz.isBusy() || atrasDe.isBusy())) {
            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    delanteDe.getCurrentPosition(), delanteIz.getCurrentPosition(),
                    atrasDe.getCurrentPosition(), atrasIz.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    private void turning(int pos1, int pos2, double power) { //pos1=encoder motores izquierda, pos2=motores derecha
        //800ticks = 90grados
        motorsSetModeSTOPANDRESET();
        motorsSetTargetPosition(pos1, pos2, pos1, pos2);
        motorsSetModeRUNTOPOSITION();
        sleep(50);

        motorsSetPower(power, -power, power, -power);

        while (opModeIsActive() && (delanteIz.isBusy() || delanteDe.isBusy() || atrasIz.isBusy() || atrasDe.isBusy())) {
            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    delanteDe.getCurrentPosition(), delanteIz.getCurrentPosition(),
                    atrasDe.getCurrentPosition(), atrasIz.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void elevadorMovimiento(int pos, double power) {
        elevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevador.setTargetPosition(pos);

        elevador.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador.setPower(power);

        while (opModeIsActive() && elevador.isBusy()) {
            telemetry.addData("Elevador", "Posición: %d", elevador.getCurrentPosition());
        }
        elevador.setPower(0);
    }

    public void garraMovimiento(int cerrarGarra) {
        if (cerrarGarra==1) {
            garra.setPower(-0.7);
            sleep(1200);
            garra.setPower(0);
        } else {
            garra.setPower(0.5);
            sleep(1200);
        }
    }

    public void actionsToBasket (int posSubirElevador, int posBajarElevador) {
        goForward(15, 0.8);
        elevadorMovimiento(posSubirElevador, 1);
        elevador.setPower(0);
        sleep(1000);
        garraMovimiento(0);
        goForward(-15, -0.8);
        elevadorMovimiento(posBajarElevador,-1);
    }
}
