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
    public partial class TaskScheduler_UnitTest
    {
        /// <summary>
        /// TaskTimingStructure Constrcutor test
        /// </summary>
        [TestMethod]
        public void TaskScheduler_UnitTest1()
        {
            TaskTimingStructure taskTimingStructure = new TaskTimingStructure(100);
            Assert.IsNotNull(taskTimingStructure);
        }

        /// <summary>
        /// TaskTimingStructure IComparable interface test: less than
        /// </summary>
        [TestMethod]
        public void TaskScheduler_UnitTest2()
        {
            TaskTimingStructure taskTimingStructure1 = new TaskTimingStructure(100);
            TaskTimingStructure taskTimingStructure2 = new TaskTimingStructure(200);
            Assert.AreEqual(taskTimingStructure1.CompareTo(taskTimingStructure2), -1);
        }

        /// <summary>
        /// TaskTimingStructure IComparable interface test: equal
        /// </summary>
        [TestMethod]
        public void TaskScheduler_UnitTest3()
        {
            TaskTimingStructure taskTimingStructure1 = new TaskTimingStructure(100);
            TaskTimingStructure taskTimingStructure2 = new TaskTimingStructure(100);
            Assert.AreEqual(taskTimingStructure1.CompareTo(taskTimingStructure2), 0);
        }

        /// <summary>
        /// TaskTimingStructure IComparable interface test: greater than
        /// </summary>
        [TestMethod]
        public void TaskScheduler_UnitTest4()
        {
            TaskTimingStructure taskTimingStructure1 = new TaskTimingStructure(200);
            TaskTimingStructure taskTimingStructure2 = new TaskTimingStructure(100);
            Assert.AreEqual(taskTimingStructure1.CompareTo(taskTimingStructure2), 1);
        }

        /// <summary>
        /// Test the ElapsedTime property
        /// </summary>
        [TestMethod]
        public void TaskScheduler_UnitTest5()
        {
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            PeriodicTask slowPeriodicTask = new PeriodicTask(thePeriodicAction: periodicAction, thePeriod: 100, theName: "slowPeriodicTask");

            TaskScheduler scheduler = new TaskScheduler(new Robot());

            scheduler.Add(slowPeriodicTask);

            scheduler.Start(asyncExecution: true);

            int expectedElapsedTime = 2134;
            Thread.Sleep(expectedElapsedTime);

            scheduler.Stop();

            // Test di uguaglianza con tolleranza di 5 millisecondi
            Assert.AreEqual(expectedElapsedTime, scheduler.ElapsedTime, 5);
        }
    }
}
