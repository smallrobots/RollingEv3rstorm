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
using System.Collections;
using System.Threading;

namespace SmallRobots.Ev3ControLib_UnitTest
{
    [TestClass]
    public class Robot_UnitTest
    {
        [TestMethod]
        public void Robot_UnitTest1()
        {
            Robot robot = new Robot();
            Assert.IsNotNull(robot);
        }
        
        /// <summary>
        /// Creates a new Periodic Action and queue it to the ActivityScheduler
        /// Just 1 PeriodicTask
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest2()
        {
            // Creates a periodic control loop
            int period = 10;
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask periodicTask = new PeriodicTask(thePeriodicAction: periodicAction,
                thePeriod: period);

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask);

            Assert.AreSame(periodicTask, robot.TaskScheduler.GetNextTask());
        }

        /// <summary>
        /// Creates a new Periodic Action and queue it to the ActivityScheduler
        /// Just 2 PeriodicTask
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest3()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask slowPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 10);
            PeriodicTask fastPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 5);

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: slowPeriodicTask);
            robot.TaskScheduler.Add(thePeriodicTask: fastPeriodicTask);

            Assert.AreSame(fastPeriodicTask, robot.TaskScheduler.TaskQueue.Values[1]);
            Assert.AreSame(slowPeriodicTask, robot.TaskScheduler.TaskQueue.Values[0]);
        }

        /// <summary>
        /// Creates a new Periodic Action and queue it to the ActivityScheduler
        /// Just 2 PeriodicTask
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest4()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask slowPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 10);
            PeriodicTask fastPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 5);
            PeriodicTask superFastPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 1);

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: slowPeriodicTask);
            robot.TaskScheduler.Add(thePeriodicTask: fastPeriodicTask);
            robot.TaskScheduler.Add(thePeriodicTask: superFastPeriodicTask);

            Assert.AreSame(superFastPeriodicTask, robot.TaskScheduler.TaskQueue.Values[2]);
            Assert.AreSame(fastPeriodicTask, robot.TaskScheduler.TaskQueue.Values[1]);
            Assert.AreSame(slowPeriodicTask, robot.TaskScheduler.TaskQueue.Values[0]);
        }

        /// <summary>
        /// Creates an empty scheduler, starts the scheduler asking for the log
        /// stops the scheduler and checks the log
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest5()
        {
            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Start(true);

            // Waits a second
            Thread.Sleep(1000);

            // Stops the scheduler
            robot.TaskScheduler.Stop();

            // Checks the log
            string log = robot.TaskScheduler.LastExecutionLog;

            Assert.AreEqual("Execution stopped by user.", log);
        }

        /// <summary>
        /// Creates an empty scheduler, checks the log without starting the scheduler
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest6()
        {
            // Creates a Robot
            Robot robot = new Robot();

            // Checks the log
            string log = robot.TaskScheduler.LastExecutionLog;

            Assert.AreEqual("", log);
        }

        /// <summary>
        /// Starts the Task Scheduler
        /// Test with just one Periodic Task
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest7()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask slowPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 100, theName: "slowPeriodicTask");
 
            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: slowPeriodicTask);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            Thread.Sleep(1050);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            int numberOfLines = robot.TaskScheduler.LastExecutionLog.Split('\n').Length;
            for (int i = 0; i < (numberOfLines-1); i ++)
            {
                double timing = Convert.ToDouble(robot.TaskScheduler.LastExecutionLog.Split('\n')[i].Split(' ')[0]);
                Assert.AreEqual(100.0d * i, timing, 3.0f);
            }
        }

        /// <summary>
        /// Starts the Task Scheduler
        /// Test with just two Periodic Task
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest8()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask slowPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 100, theName: "slowPeriodicTask");
            PeriodicTask fastPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 50, theName: "fastPeriodicTask");

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: slowPeriodicTask);
            robot.TaskScheduler.Add(thePeriodicTask: fastPeriodicTask);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            Thread.Sleep(1025);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            string executionLog = robot.TaskScheduler.LastExecutionLog;

            // Estrae il log di slowPeriodicTask
            ArrayList slowPeriodicTaskLog = new ArrayList();
            ArrayList fastPeriodicTaskLog = new ArrayList();
            foreach (string line in robot.TaskScheduler.LastExecutionLog.Split('\n'))
            {
                if (line.Contains("slowPeriodicTask"))
                {
                    slowPeriodicTaskLog.Add(line);
                }
                if (line.Contains("fastPeriodicTask"))
                {
                    fastPeriodicTaskLog.Add(line);
                }
            }

            // 10 slow executions mixed with
            Assert.AreEqual(11, slowPeriodicTaskLog.Count);
            // 20 slow executions
            Assert.AreEqual(21, fastPeriodicTaskLog.Count);

            // Test for accuracy
            for (int i = 0; i< 10; i++)
            {
                double timing = Convert.ToDouble(((string) slowPeriodicTaskLog[i]).Split(' ')[0]);
                Assert.AreEqual(100.0d * i, timing, 2.0f);
            }
            for (int i = 0; i < 20; i++)
            {
                double timing = Convert.ToDouble(((string)fastPeriodicTaskLog[i]).Split(' ')[0]);
                Assert.AreEqual(50.0d * i, timing, 2.0f);
            }
        }

        /// <summary>
        /// Starts the Task Scheduler
        /// Test with 4 Periodic Tasks
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest9()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask periodicTask1 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 100, theName: "periodicTask1");
            PeriodicTask periodicTask2 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 50, theName: "periodicTask2");
            PeriodicTask periodicTask3 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 50, theName: "periodicTask3");
            PeriodicTask periodicTask4 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 25, theName: "periodicTask4");

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask1);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask2);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask3);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask4);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            Thread.Sleep(10010);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            string executionLog = robot.TaskScheduler.LastExecutionLog;

            // Estrae il log di slowPeriodicTask
            ArrayList periodicTask1Log = new ArrayList();
            ArrayList periodicTask2Log = new ArrayList();
            ArrayList periodicTask3Log = new ArrayList();
            ArrayList periodicTask4Log = new ArrayList();
            foreach (string line in robot.TaskScheduler.LastExecutionLog.Split('\n'))
            {
                if (line.Contains("periodicTask1"))
                {
                    periodicTask1Log.Add(line);
                }
                if (line.Contains("periodicTask2"))
                {
                    periodicTask2Log.Add(line);
                }
                if (line.Contains("periodicTask3"))
                {
                    periodicTask3Log.Add(line);
                }
                if (line.Contains("periodicTask4"))
                {
                    periodicTask4Log.Add(line);
                }
            }

            // Tests number of executions
            Assert.AreEqual(101, periodicTask1Log.Count);
            Assert.AreEqual(201, periodicTask2Log.Count);
            Assert.AreEqual(201, periodicTask3Log.Count);
            Assert.AreEqual(401, periodicTask4Log.Count);

            // Test for accuracy
            for (int i = 0; i < 100; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask1Log[i]).Split(' ')[0]);
                Assert.AreEqual(100.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 201; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask2Log[i]).Split(' ')[0]);
                Assert.AreEqual(50.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 201; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask3Log[i]).Split(' ')[0]);
                Assert.AreEqual(50.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 401; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask4Log[i]).Split(' ')[0]);
                Assert.AreEqual(25.0d * i, timing, 10.0f);
            }
        }

        /// <summary>
        /// Starts the Task Scheduler
        /// Test with 4 Periodic Tasks
        /// </summary>
        [TestMethod]
        public void Robot_UnitTest10()
        {
            // Creates a periodic control loop
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask periodicTask1 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 100, theName: "periodicTask1");
            PeriodicTask periodicTask2 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 50, theName: "periodicTask2");
            PeriodicTask periodicTask3 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 50, theName: "periodicTask3");
            PeriodicTask periodicTask4 = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 25, theName: "periodicTask4");

            // Creates a Robot
            Robot robot = new Robot();

            // Append the periodic control loop to the robot scheduler
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask1);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask2);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask3);
            robot.TaskScheduler.Add(thePeriodicTask: periodicTask4);

            // Starts the TaskScheduler with log
            robot.TaskScheduler.Start(true, true);

            // Waits 1s
            int expectedElapsedTime = 10010;
            Thread.Sleep(expectedElapsedTime);

            // Stops the TaskScheduler
            robot.TaskScheduler.Stop();

            // Scheduler testato con tolleranza di 2ms.
            string executionLog = robot.TaskScheduler.LastExecutionLog;

            // Estrae il log di slowPeriodicTask
            ArrayList periodicTask1Log = new ArrayList();
            ArrayList periodicTask2Log = new ArrayList();
            ArrayList periodicTask3Log = new ArrayList();
            ArrayList periodicTask4Log = new ArrayList();
            foreach (string line in robot.TaskScheduler.LastExecutionLog.Split('\n'))
            {
                if (line.Contains("periodicTask1"))
                {
                    periodicTask1Log.Add(line);
                }
                if (line.Contains("periodicTask2"))
                {
                    periodicTask2Log.Add(line);
                }
                if (line.Contains("periodicTask3"))
                {
                    periodicTask3Log.Add(line);
                }
                if (line.Contains("periodicTask4"))
                {
                    periodicTask4Log.Add(line);
                }
            }

            // Tests number of executions
            Assert.AreEqual(101, periodicTask1Log.Count);
            Assert.AreEqual(201, periodicTask2Log.Count);
            Assert.AreEqual(201, periodicTask3Log.Count);
            Assert.AreEqual(401, periodicTask4Log.Count);

            // Test for accuracy
            for (int i = 0; i < 100; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask1Log[i]).Split(' ')[0]);
                Assert.AreEqual(100.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 201; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask2Log[i]).Split(' ')[0]);
                Assert.AreEqual(50.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 201; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask3Log[i]).Split(' ')[0]);
                Assert.AreEqual(50.0d * i, timing, 10.0f);
            }
            for (int i = 0; i < 401; i++)
            {
                double timing = Convert.ToDouble(((string)periodicTask4Log[i]).Split(' ')[0]);
                Assert.AreEqual(25.0d * i, timing, 10.0f);
            }

            // Test elapsed time with 5ms tolerance
            Assert.AreEqual(expectedElapsedTime, robot.ElapsedTime, 5);
        }
    }
}
