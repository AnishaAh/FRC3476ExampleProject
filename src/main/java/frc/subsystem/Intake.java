package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.OrangeUtility;

//Test

import static frc.utility.Pneumatics.getPneumaticsHub;

public final class Intake extends AbstractSubsystem{

    private static Intake instance = new Intake();
    private final Solenoid intakeSol;
    private  double allowIntakeRunTime = Double.MAX_VALUE;

    private TalonFX intakeMotorFalcon;
    public Intake() {
        super(-1, 4);
        intakeSol = getPneumaticsHub().makeSolenoid(3);
        intakeMotorFalcon = new TalonFX(40);
        intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 97);
        intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 101);
        intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 103);
        intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_3_General, 23);
        intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_4_Advanced, 29);
        intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 547);
        intakeMotorFalcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 120, 0.01), 1000);
        intakeMotorFalcon.configOpenloopRamp(0.2, 1000);
    }

    @Override
    public void selfTest() {
        //TO DO
    }

    @Override
    public void logData() {
        //To do
    }

    @Override
    public void close() throws Exception {

    }

    public IntakeSolState getIntakeSolState() {return intakeSol.get() ? IntakeSolState.OPEN : IntakeSolState.CLOSE;}

    public enum IntakeSolState {
        OPEN, CLOSE;
        public IntakeSolState invert() {return this == OPEN ? CLOSE : OPEN;}
    }

    public synchronized void setIntakeSolState(IntakeSolState intakeSolState) {
        SmartDashboard.putString("Intake State", intakeSolState.toString());
        switch (intakeSolState) {
            case  OPEN:
                if (Timer.getFPGATimestamp() + Constants.INTAKE_OPEN_TIME < allowIntakeRunTime) {
                    allowIntakeRunTime = Timer.getFPGATimestamp() + Constants.INTAKE_OPEN_TIME;
                }
                intakeSol.set(true);
                break;
            case CLOSE:
                intakeSol.set(false);
                allowIntakeRunTime = Double.MAX_VALUE;
        }
    }

    public enum IntakeState {
        INTAKE, EJECT, SLOW_EJECT, OFF
    }

    private IntakeState wantedIntakeState = IntakeState.OFF;

    public synchronized void setWantedIntakeState(IntakeState intakeState) {
        wantedIntakeState = intakeState;
    }

    private void setIntakeMotor(double speed) {
        intakeMotorFalcon.set(ControlMode.PercentOutput, speed);

    }

    private void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case INTAKE:
                setIntakeMotor(Constants.INTAKE_SPEED);
                break;
            case EJECT:
                setIntakeMotor(-Constants.INTAKE_SPEED);
                break;
            case SLOW_EJECT:
                setIntakeMotor(-Constants.INTAKE_SPEED / 2.5);
                break;
            case OFF:
                setIntakeMotor(0);
                break;
        }
    }

}
