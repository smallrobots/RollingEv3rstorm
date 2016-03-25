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

namespace SmallRobots.Ev3ControLib_UnitTest
{
    [TestClass]
    public class Task_UnitTest
    {
        /// <summary>
        ///  ContinuousControlLoop Default constructor test
        /// </summary>
        [TestMethod]
        public void Task_UnitTest1()
        {
            ContinuousTask continuousControlLoop = new ContinuousTask();
            Assert.IsNotNull(continuousControlLoop);
        }

        /// <summary>
        /// PeriodicControlLoop Constructor with parameter test
        /// </summary>
        [TestMethod]
        public void Task_UnitTest2()
        {
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            int period = 10;

            PeriodicTask periodicControlLoop = new PeriodicTask(thePeriodicAction: periodicAction,
                thePeriod: period);

            Assert.IsNotNull(periodicControlLoop);
            Assert.ReferenceEquals(periodicControlLoop.Action, periodicAction);
            Assert.AreEqual(periodicControlLoop.Period, period);
        }

        /// PeriodicControlLoop Constructor with parameter test
        /// </summary>
        [TestMethod]
        public void Task_UnitTest3()
        {
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            int period = 10;

            PeriodicTask periodicControlLoop = new PeriodicTask();
            periodicControlLoop.Action = periodicAction;
            periodicControlLoop.Period = period;

            Assert.IsNotNull(periodicControlLoop);
            Assert.AreSame(periodicControlLoop.Action, periodicAction);
            Assert.AreEqual(periodicControlLoop.Period, period);
        }

        /// PeriodicControlLoop Constructor with parameter test
        /// </summary>
        [TestMethod]
        public void Task_UnitTest4()
        {
            Action<Robot> periodicAction = new Action<Robot>((Robot robotState) => { });
            int period = 10;
            string name = "TestName";

            PeriodicTask periodicControlLoop = new PeriodicTask(thePeriodicAction: periodicAction,
                                                                thePeriod: period,
                                                                theName: name);

            Assert.IsNotNull(periodicControlLoop);
            Assert.AreSame(periodicControlLoop.Action, periodicAction);
            Assert.AreEqual(periodicControlLoop.Period, period);
            Assert.AreEqual(periodicControlLoop.Name, name);
        }
    }
}
