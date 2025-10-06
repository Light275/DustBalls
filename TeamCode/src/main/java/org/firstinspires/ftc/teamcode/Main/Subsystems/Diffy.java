package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Main.Utils.PID;

public class Diffy {

    private CRServo diffyL, diffyR;
    private DcMotorEx encL, encR; // encoders measuring servo positions

    private PID pidL, pidR; // PIDs are internal to Diffy

    private double targetGantryTicks = 0;
    private double targetTurretTicks = 0;

    private double toleranceTicks = 5; // configurable

    public Diffy(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "intake");
        encR = hardwareMap.get(DcMotorEx.class, "leftBack");

        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize internal PID controllers
        pidL = new PID(0.01, 0, 0.0005); // TODO: Tune PID Parameters
        pidR = new PID(0.01, 0, 0.0005);

        pidL.setOutputLimits(-1, 1);
        pidR.setOutputLimits(-1, 1);

        pidL.setEnabled(true);
        pidR.setEnabled(true);
    }

    /** Set target gantry position in ticks */
    public void setGantryTarget(double gantryTicks) {
        targetGantryTicks = gantryTicks;
    }

    /** Set target turret rotation in ticks */
    public void setTurretTarget(double turretTicks) {
        targetTurretTicks = turretTicks;
    }

    /** Set tolerance for being "at target" */
    public void setTolerance(double ticks) {
        toleranceTicks = ticks;
    }

    /** Update CR servo powers using internal PIDs */
    public void update() {
        double leftTarget = targetGantryTicks + targetTurretTicks;
        double rightTarget = targetGantryTicks - targetTurretTicks;

        double leftCurrent = encL.getCurrentPosition();
        double rightCurrent = encR.getCurrentPosition();

        double powerL = pidL.update(leftCurrent, leftTarget);
        double powerR = pidR.update(rightCurrent, rightTarget);

        diffyL.setPower(Range.clip(powerL, -1, 1));
        diffyR.setPower(Range.clip(powerR, -1, 1));
    }

    /** Returns true if both servos are within tolerance */
    public boolean atTarget() {
        double errorL = Math.abs(targetGantryTicks + targetTurretTicks - encL.getCurrentPosition());
        double errorR = Math.abs(targetGantryTicks - targetTurretTicks - encR.getCurrentPosition());
        return errorL <= toleranceTicks && errorR <= toleranceTicks;
    }

    public void disablePID() {
        pidL.setEnabled(false);
        pidR.setEnabled(false);
        diffyL.setPower(0);
        diffyR.setPower(0);
    }

    public void enablePID() {
        pidL.setEnabled(true);
        pidR.setEnabled(true);
    }


    public void setPID(double kP, double kI, double kD) {
        pidL.setPID(kP, kI, kD);
        pidR.setPID(kP, kI, kD);
    }
}
