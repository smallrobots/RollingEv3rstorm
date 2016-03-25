//////////////////////////////////////////////////////////////////////////////////////////////////
// Ev3 Control Library (Ev3ControlLib)                                                          //
// Version 1.0                                                                                  //
//                                                                                              //
// Happily shared under the MIT License (MIT)                                                   //
//                                                                                              //
// Copyright(c) 2016 SmallRobots.it                                                             //
//                                                                                              //
// Permission is hereby granted, free of charge, to any person obtaining                        //
//a copy of this software and associated documentation files (the "Software"),                  //
// to deal in the Software without restriction, including without limitation the rights         //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies             //
// of the Software, and to permit persons to whom the Software is furnished to do so,           //      
// subject to the following conditions:                                                         //
//                                                                                              //
// The above copyright notice and this permission notice shall be included in all               //
// copies or substantial portions of the Software.                                              //
//                                                                                              //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,          //
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR     //
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE           //
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,          //
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE        //
// OR OTHER DEALINGS IN THE SOFTWARE.                                                           //
//////////////////////////////////////////////////////////////////////////////////////////////////

using Microsoft.VisualStudio.TestTools.UnitTesting;
using SmallRobots.Ev3ControlLib;
using System;
using System.Threading;

namespace Ev3ControLib_UnitTest
{
    [TestClass]
    public partial class PIDController_UnitTest
    {
        /// <summary>
        /// Default constructor test
        /// </summary>
        [TestMethod]
        public void PIDController_UnitTest1()
        {
            PIDController pid = new PIDController();
            Assert.IsNotNull(pid);
        }

        /// <summary>
        /// Constructor with parameters
        /// </summary>
        [TestMethod]
        public void PIDController_UnitTest2()
        {
            PIDController pid = new PIDController(10);
            Assert.IsNotNull(pid);
        }

        /// <summary>
        /// Test the algorithm execution (not the performances)
        /// </summary>
        [TestMethod]
        public void PIDController_UnitTest3()
        {
            PIDController pid = new PIDController(100);

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: pid);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            Thread.Sleep(1050);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            int numberOfLines = robot.TaskScheduler.LastExecutionLog.Split('\n').Length;
            for (int i = 0; i < (numberOfLines - 1); i++)
            {
                double timing = Convert.ToDouble(robot.TaskScheduler.LastExecutionLog.Split('\n')[i].Split(' ')[0]);
                Assert.AreEqual(100.0d * i, timing, 3.0f);
            }

        }

        /// <summary>
        /// Test that the algorithm does something
        /// </summary>
        [TestMethod]
        public void PIDController_UnitTest4()
        {
            PIDController pid = new PIDController(10);
            pid.Kp = 10.0f;
            pid.Ki = 5.0f;
            pid.Kd = 0.0f;
            pid.MaxPower = 100;
            pid.MinPower = -100;
            pid.SetPoint = 3.0f;

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: pid);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            Thread.Sleep(1050);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            int numberOfLines = robot.TaskScheduler.LastExecutionLog.Split('\n').Length;
            for (int i = 0; i < (numberOfLines - 1); i++)
            {
                double timing = Convert.ToDouble(robot.TaskScheduler.LastExecutionLog.Split('\n')[i].Split(' ')[0]);
                Assert.AreEqual(10.0d * i, timing, 3.0f);
            }

            // Test Output signal out of scale
            Assert.AreEqual(pid.MaxPower, pid.OutputSignal);

        }
    }
}
