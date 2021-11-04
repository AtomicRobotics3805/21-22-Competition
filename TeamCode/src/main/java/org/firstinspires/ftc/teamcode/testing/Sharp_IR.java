/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Sharp_IR", group="Linear Opmode")

public class Sharp_IR extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    AnalogInput mb1220;
    Servo scanner;
    double voltage;
    boolean position_a;
    boolean position_b;
    boolean position_c;
    @Override
    public void runOpMode() {

        mb1220 = hardwareMap.get(AnalogInput.class, "mb1220");
        scanner = hardwareMap.get(Servo.class, "scanner");

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (runtime.seconds() < 1) {
                scanner.setPosition(0.9);
            }
            voltage = mb1220.getVoltage();
            if (voltage > 0.015) {
                position_a = true;
            }
            runtime.reset();
            while (runtime.seconds() < 1){
               scanner.setPosition(0.9);
            }
            voltage = mb1220.getVoltage();
            if (voltage < 0.015 && position_a == false) {
                position_b = true;
            } else if (position_a == false) {
                position_c = true;
            }
            while (opModeIsActive()) {
                telemetry.addData("Position_A", position_a);
                telemetry.addData("Position_B", position_b);
                telemetry.addData("Position_C", position_c);
                telemetry.addData("Voltage", voltage);
                telemetry.update();
            }
        }
        }
    }

