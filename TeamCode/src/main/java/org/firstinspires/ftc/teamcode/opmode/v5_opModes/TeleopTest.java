package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Deprecated
@TeleOp(name="TeleopTest F")
@Config
public class TeleopTest extends OpMode {
    List<Servo> servos = new ArrayList<>(4);
    private V5 robot;
    boolean sugadorEstado= false ;
    double turnAlvo;
    double correction;
    double turn;
    double heading;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static double powerDriveOut = -1, powerStrafeOut = -1,powerDriveInt  = 1,powerStrafeInt = 1,kp =0.1, tempoIncercia = 0.1;

    private double delayMode = 0.3, cooldownMode = 0, cooldownInercia = 0, lastUpdateTime = 0, updateInterval = 0.050, loopTime = 0, lastTime = 0; // 50ms;



    public boolean needToOutake = false;

    @Override
    public  void init() {
        robot = this.createRobot(hardwareMap);
        Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));
        turnAlvo = -90;

        robot.md.pose = initialPose;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //JsonFileCreator.createJsonFile();

        //robot.imu.initialize(robot.parameters);

    }

    /*********************************************************************************************************************************************************************\
     Depois que o código for executado, o código dentro desse metodo será executado continuamente até que o botão START seja pressionado na estação do driver.init()
     \********************************************************************************************************************************************************************/
    @Override
    public void init_loop(){
        calculaTempoDeLoop();
        this.gerenciarModo(robot,gamepadEx2);
        this.gerenciarModoSuccao(robot,gamepadEx2);
        telemetry.addData(" MODO DE PONTUAÇÃO", robot.v5Mode);
        telemetry.addData(" TEMPO DE EXECUÇÃO", getRuntime());
        telemetry.addData("Distancia horizontal", robot.intakeInferior.linearHorizontalMotor.colorSensor.getDistance());
        telemetry.addData("Horizontal ta sendo visto", robot.intakeInferior.linearHorizontalMotor.sensorIsDetectingHorintontal());
        if (getRuntime() - lastUpdateTime >= updateInterval) {
            telemetry.update();
            lastUpdateTime = getRuntime();
        }
        if(!IntakeSuccao.toggle){
            telemetry.addLine("Manual");
        }
        else if(IntakeSuccao.toggle){
            telemetry.addLine("Toggle");
        }

    }


    @Override
    public void  start() {
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
        robot.intakeInferior.goToInitial_goToReadyTransfer(robot.carteiro, getRuntime());
        robot.intakeInferior.linearHorizontalMotor.reset();
        if(robot.v5Mode == V5Modes.SPECIMEN){
            robot.outtakeIntakeSuperior.goToInitial(robot.carteiro, getRuntime());
        }
        if(robot.v5Mode == V5Modes.SAMPLE){
            robot.outtakeIntakeSuperior.goToReadyTransfer(robot.carteiro, 0, getRuntime());
        }



        resetRuntime();

    }
    @Override
    public void loop() {

        calculaTempoDeLoop();
        this.gerenciarModo(robot,gamepadEx2);

        this.robotCentricDrive(robot,gamepadEx1);
        if(robot.v5Mode == V5Modes.SPECIMEN){
            this.bindsChamber(robot,gamepadEx2, robot.carteiro);
        } else if (robot.v5Mode == V5Modes.SAMPLE) {
            this.bindsSample(robot,gamepadEx2,robot.carteiro);
        }
        this.linearHorizontal(robot.carteiro,robot.intakeInferior.linearHorizontalMotor,gamepadEx2);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx2, robot.carteiro);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx2,robot.carteiro);
        this.IntakeSuccao(robot,robot.intakeInferior.intakeSuccao,robot.carteiro,gamepadEx2);
        this.garraSuperior(robot.outtakeIntakeSuperior.braco,robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx2);
        this.hang(gamepadEx1, robot.carteiro);
        //this.verificarsample(robot,robot.carteiro, "Azul");

        this.runActions(robot.carteiro);

        telemetry.addData(" MODO DE PONTUAÇÃO", robot.v5Mode);
        telemetry.addData(" TEMPO DE EXECUÇÃO", getRuntime());
        //telemetry.addData("yaw",robot.md.lazyImu.get().getRobotYawPitchRollAngles().getYaw());


        ;
        telemetry.addData("motor frontal direito",robot.md.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor traseiro direito",robot.md.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor frontal esquerdo",robot.md.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor traseiro esquerdo",robot.md.leftBack.getCurrent(CurrentUnit.AMPS));





        if (getRuntime() - lastUpdateTime >= updateInterval) {
            telemetry.update();
            lastUpdateTime = getRuntime();
        }

    }

    @Override
    public void stop() {
        robot.carteiro.saveLogsToJson();
    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {

        return new V5(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad) {
        //todo: botao para andar diagonal sem alteração no heading
        robot.md.updatePoseEstimate();
        telemetry.addData("Heading (Graus)", heading);
        telemetry.addData("turn alvo:",turnAlvo);
        telemetry.addData("correction",correction);
        double drive = Range.clip(gamepad.getLeftY(), -1, 1);


        //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

        double strafe = Range.clip(-gamepad.getLeftX(), -1, 1);
        //if (gamepad1.right_stick_button) strafe = -0.6;
        //if (gamepad1.left_stick_button) strafe = 0.6;
        //if ( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);
        heading = Math.toDegrees(Math.atan2(robot.md.pose.heading.imag, robot.md.pose.heading.real));

        if(Math.abs(gamepad.getRightX()) > 0.4 ){
            turn = Range.clip(-gamepad.getRightX(),-1,1);
            turnAlvo = heading;
            cooldownInercia = tempoIncercia + getRuntime();
        }
        else if(robot.intakeInferior.intakeSuccao.sugarAngulationStates ==SugarAngulationStates.INTAKE||robot.intakeInferior.intakeSuccao.sugarAngulationStates== SugarAngulationStates.READY_TOINTAKE){
            correction=0;
        }
        else if(getRuntime() >= cooldownInercia) {
            correction = (turnAlvo - heading) * kp;
            turn = Range.clip(correction,-1,1);
        }
        else{
            turnAlvo = heading;
        }


        if (gamepad1.left_trigger > 0) {
            strafe = -gamepad1.left_trigger * 1;
        }

        if (gamepad1.right_trigger > 0) {
            strafe = gamepad1.right_trigger * 1;
        }
        if(strafe != 0 && turn == 0){

        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            drive = powerDriveOut;
            strafe = powerStrafeOut;
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            drive = powerDriveInt;
            strafe = powerStrafeInt;
        }
        /*
        if(gamepad.getButton(GamepadKeys.Button.A)){
            Actions.runBlocking(
                    new SequentialAction(
                            robot.md.actionBuilder(robot.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-5, -26, Math.toRadians(-90)), Math.toRadians(90))
                                    .build()
                    )
            );
        }*/

        if(gamepad.getButton(GamepadKeys.Button.A)){
            Actions.runBlocking(
                    new SequentialAction(
                            robot.md.actionBuilder(robot.md.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(47, -50, Math.toRadians(-90)), Math.toRadians(-90))
                                    .build()
                    )
            );
        }
        turn = Range.clip(turn / 1.3, -0.7, 0.7);

        robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));



    }
    private void testar1servo(V5 robot, GamepadEx gamepad,OrdersManager carteiro) {
    }
    private void hang(GamepadEx gamepad, OrdersManager carteiro) {
        if(gamepad.getButton(GamepadKeys.Button.X)) {
            robot.goReadytoHang(carteiro, getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.B)) {
            robot.goToHang(carteiro, getRuntime());
        }
    }
    private void bindsChamber(V5 robot, GamepadEx gamepad, OrdersManager carteiro) {

        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(carteiro, getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.intakeInferior.gerenciadorIntakSpecimen(carteiro,getRuntime());


        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.B)){



            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToInitial(robot.carteiro, getRuntime());
        }

    }//todo okey
    private void bindsSample(V5 robot, GamepadEx gamepad, OrdersManager carteiro){
        if(gamepad.getButton(GamepadKeys.Button.B)){


            robot.outtakeIntakeSuperior.goToReadyTransfer(carteiro, 0, getRuntime());
            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro, getRuntime());

        }
        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.outtakeIntakeSuperior.CorreProTransfer(carteiro,getRuntime()
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.intakeInferior.gerenciadorIntakeSample(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.outtakeIntakeSuperior.goToOuttakeBASKET(carteiro,0.2,getRuntime());
        }
    }//todo errado

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad, OrdersManager carteiro)  {
        vertical.PIDF();
        vertical.monitor(telemetry);
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            vertical.downSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.BACK)){

            vertical.reset();
        }

    }//todo okey
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
        if(gamepad.getLeftY() > 0){
            robot.outtakeIntakeSuperior.braco.upSetPoint(gamepad.getLeftY());
        }
        else if(gamepad.getLeftY() < 0){
            robot.outtakeIntakeSuperior.braco.downSetPoint(gamepad.getLeftY());
        }
        braco.monitor(telemetry);
    }
    private void linearHorizontal(OrdersManager carteiro, LinearHorizontalMotor horizontal, GamepadEx gamepad)  {
        horizontal.monitor(telemetry,"horizontal inferior");
        horizontal.PID();
        // if(gamepad.getButton(GamepadKeys.Button.A)){
        //  carteiro.addOrder(horizontal.horizontalGoTo(140),0,"horizonte",getRuntime());
        // }
        //if(gamepad.getButton(GamepadKeys.Button.B)){
        //carteiro.addOrder(horizontal.horizontalGoTo(0),0,"horizonte",getRuntime());
        // }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            horizontal.downSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            horizontal.upSetPoint();
        }

        //if(gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
        // horizontal.reset();
        //}


    }
    private void garraSuperior(BracoGarraSuperior bracoGarraSuperior,GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
        garra.monitor(telemetry);
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime(),bracoGarraSuperior.bracoGarraSuperiorState ),0.0,"garra superior",getRuntime());

        }
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDeRotacaoDaGarraNoTeleop(getRuntime(), robot.v5Mode),0.0,"garra superior",getRuntime());

        }
        if(gamepad.getRightY() > 0){
            garra.upSetPoint(gamepad.getRightY());
        }
        else if(gamepad.getRightY() < 0){
            garra.downSetPoint(gamepad.getRightY());
        }

        if(gamepad.getRightX() > 0.5){
            garra.upSetPointRot(gamepad.getRightX());
        }
        else if(gamepad.getRightX() < -0.5  ){
            garra.downSetPointRot(gamepad.getRightX());
        }
    }
    private void IntakeSuccao(V5 robot, IntakeSuccao intakeSuccao, OrdersManager carteiro, GamepadEx gamepad){
        intakeSuccao.monitor(telemetry);
        if(IntakeSuccao.monitor){
            telemetry.addData("Tem sample", robot.intakeInferior.matchColor.verifyPositionSampleToTransfer());
            telemetry.addData("red sample", robot.intakeInferior.matchColor.isRed(robot.intakeInferior.intakeSuccao));
            telemetry.addData("blue sample", robot.intakeInferior.matchColor.isBlue(robot.intakeInferior.intakeSuccao));
            telemetry.addData("yellow sample",robot.intakeInferior.matchColor.isYellow(robot.intakeInferior.intakeSuccao));
            telemetry.addData("Somared", robot.intakeInferior.matchColor.getRed(robot.intakeInferior.intakeSuccao.colorSensorSugar.getRed()));
            telemetry.addData("Somazul", robot.intakeInferior.matchColor.getBlue(robot.intakeInferior.intakeSuccao.colorSensorSugar.getBlue()));
            telemetry.addData("Somagreen", robot.intakeInferior.matchColor.getGreen(robot.intakeInferior.intakeSuccao.colorSensorSugar.getGreen()));
            telemetry.addData("Distancia", robot.intakeInferior.intakeSuccao.colorSensorSugar.getDistance());
            telemetry.addData("media", robot.intakeInferior.matchColor.media);

        }
        if(gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            intakeSuccao.angulacao.getController().pwmDisable();

        }


        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > IntakeSuccao.pontoAtiv || gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > IntakeSuccao.pontoAtiv){
            if(IntakeSuccao.toggle){
                robot.intakeInferior.intakeSuccao.gerenciadorDoSugador(gamepad, getRuntime());
            }

        }
        if(!IntakeSuccao.toggle){
            robot.intakeInferior.intakeSuccao.gerenciadorDoSugadorManual(gamepad, getRuntime());
        }



    }//todo okey
    private void fullAutoOuttakeChamber(V5 robot , GamepadEx gamepad,OrdersManager carteiro){
        /*if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.md.updatePoseEstimate();
            Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(

                            //robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(),
                            robot.MoveOuttake(robot)

                    ),
                    robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()

                )
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.md.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            robot.outtakeIntakeSuperior.goToIntakeChamber2(),
                            robot.MoveIntake(robot)
                    )
            );
        }*/

    }
    private void gerenciarModo(V5 robot,GamepadEx gamepadEx) {
        if(gamepadEx.getButton(GamepadKeys.Button.START)) {
            if (getRuntime() >= cooldownMode) {
                cooldownMode = getRuntime() + delayMode;

                if (this.robot.v5Mode == V5Modes.SPECIMEN) {
                    robot.v5Mode = V5Modes.SAMPLE;
                } else {
                    robot.v5Mode = V5Modes.SPECIMEN;
                }

            }
        }
    }
    private void gerenciarModoSuccao(V5 robot,GamepadEx gamepadEx) {
        if(gamepadEx.getButton(GamepadKeys.Button.BACK)) {
            if (getRuntime() >= cooldownMode) {
                cooldownMode = getRuntime() + delayMode;

                if (IntakeSuccao.toggle) {
                    IntakeSuccao.toggle = false;
                } else {
                    IntakeSuccao.toggle = true;
                }

            }
        }
    }

    private void calculaTempoDeLoop() {
        loopTime = getRuntime() - lastTime;
        telemetry.addData("Latência código", String.format("%.0f",loopTime*1000) + "ms");
        lastTime = getRuntime();
    }

    private void verificarsample(V5 robot, OrdersManager carteiro, String ladoAlianca){
        if(robot.v5Mode == V5Modes.SAMPLE && ladoAlianca.equals("Azul")){
            if(robot.intakeInferior.matchColor.isRed(robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        //MIGUEL DESCOMENTE ISSO EU COMENTEI PRA TESTAR
        if(robot.v5Mode == V5Modes.SPECIMEN && ladoAlianca.equals("Azul")){
            if(robot.intakeInferior.matchColor.isRed(robot.intakeInferior.intakeSuccao) || robot.intakeInferior.matchColor.isYellow(robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        if(robot.v5Mode == V5Modes.SAMPLE && ladoAlianca.equals("Red")){
            if(robot.intakeInferior.matchColor.isBlue( robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        //MIGUEL DESCOMENTE ISSO EU COMENTEI PRA TESTAR
        if(robot.v5Mode == V5Modes.SPECIMEN && ladoAlianca.equals("Red")){
            if(robot.intakeInferior.matchColor.isBlue( robot.intakeInferior.intakeSuccao) || robot.intakeInferior.matchColor.isYellow(  robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
    }



}