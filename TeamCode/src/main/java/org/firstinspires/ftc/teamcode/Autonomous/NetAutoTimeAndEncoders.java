package org.firstinspires.ftc.teamcode.Autonomous;

//500ms en avanzar 1 tapete

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AutoTiemposIZQUIERDA")
public class NetAutoTimeAndEncoders extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz;
    private DcMotor delanteDe;
    private DcMotor atrasIz;
    private DcMotor atrasDe;
    private DcMotor elevador;
    private CRServo garra;
    private DcMotor Intake;

    //private boolean garraAbierta = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
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

        garra.setPower(0);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()){
            //AUTONOMO POR TIEMPOS PROGRAMA PRINCIPAL
            garraMovimiento(1);
            adelanteAtras(620, 1);
            sleep(100);
            turnEncoder(-1440, 1440, 1);

            elevadorMovimiento(6000, 0.9);
            adelanteAtras(350, 0.8);
            garraMovimiento(0);
            sleep(1000);

            adelanteAtras(200, -1);

            elevadorMovimiento(5400, -0.9); //SIRVE NO TOCAR
        }
    }

    public void adelanteAtras(long time, double power) {
        motorsSetModeUSEWITHOUTENCODER();
        delanteIz.setPower(power);
        delanteDe.setPower(power);
        atrasIz.setPower(power);
        atrasDe.setPower(power);

        sleep(time);

        stopMotors();
    }


    public void strafe(long time, double power, int leftRight) {
        if(leftRight==1){ //si el primer parametro de la funcion es 1, strafea a la derecha
            delanteIz.setPower(power+0.3);
            delanteDe.setPower(-power);
            atrasIz.setPower(-power);
            atrasDe.setPower(power);
        } else{ //si el primer parametro de la funcion es 0, strafea a la izquierda
            delanteIz.setPower(-power-0.3);
            delanteDe.setPower(power);
            atrasIz.setPower(power);
            atrasDe.setPower(-power);
        }

        sleep(time);

        stopMotors();
    }

    public void motorsSetModeUSEWITHOUTENCODER(){
        delanteIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    private void turnEncoder(int pos1, int pos2, double power) { //pos1=encoder motores izquierda, pos2=motores derecha
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

    public void stopMotors() {
        delanteIz.setPower(0);
        delanteDe.setPower(0);
        atrasIz.setPower(0);
        atrasDe.setPower(0);
    }

    public void elevadorMovimiento(long time, double power) {
        elevador.setPower(power);

        sleep(time);

        elevador.setPower(0);
    }

    public void garraMovimiento(int cerrarGarra) {
        if (cerrarGarra==1) {
            garra.setPower(0.01);
            sleep(500);
            garra.setPower(0);
        } else {
            garra.setPower(-0.7);
            sleep(500);
            garra.setPower(0);
        }
    }

}
