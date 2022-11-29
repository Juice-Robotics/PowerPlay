/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class TestAuton extends LinearOpMode {

    Robot robot;
    Pose2d startPose = new Pose2d(36, 58, Math.toRadians(-90));

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, true);
        robot.drive.setPoseEstimate(startPose);

        waitForStart();

            TrajectorySequence fullCycles = robot.drive.trajectorySequenceBuilder(startPose)
                    .forward(500)
//                    .splineTo(new Vector2d(-35, 0), 270)
//                    .splineToConstantHeading(new Vector2d(-61, 11), 270)
//                    .splineToConstantHeading(new Vector2d(-61, 11), 270)
//                    .splineToConstantHeading(new Vector2d(-35, 0), 270)
//                    .splineToConstantHeading(new Vector2d(-61, 11), 270)
//                    .splineToConstantHeading(new Vector2d(-35, 0), 270)
//                    .splineToConstantHeading(new Vector2d(-61, 11), 270)
//                    .splineToConstantHeading(new Vector2d(-35, 0), 270)
//                    .splineToConstantHeading(new Vector2d(-61, 11), 270)
//                    .splineToConstantHeading(new Vector2d(-35, 0), 270)
//                    .splineTo(new Vector2d(-30, 11), Math.toRadians(270))
                    .build();

            telemetry.addData("robot positiion", robot.drive.getPoseEstimate());
            telemetry.update();
            robot.drive.followTrajectorySequence(fullCycles);
    }
}