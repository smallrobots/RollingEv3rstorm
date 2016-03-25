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


namespace SmallRobots.Ev3ControlLib
{
    public partial class Robot
    {
        #region Fields
        /// <summary>
        /// Scheduler of the period and continuous actions defined for this robot
        /// </summary>
        protected TaskScheduler taskScheduler;
        #endregion

        #region Properties
        /// <summary>
        /// Elapsed time since the start of the robot activity
        /// </summary>
        public long ElapsedTime
        {
            get
            {
                return taskScheduler.ElapsedTime;
            }
        }

        /// <summary>
        /// Gets the scheduler of the periodic and the continuous action defined
        /// for this robot
        /// </summary>
        public TaskScheduler TaskScheduler
        {
            get
            {
                return taskScheduler;
            }
        }
        #endregion

        #region Constructors
        /// <summary>
        /// Constructs an empty robot
        /// </summary>
        public Robot()
        {
            // Fields initialization
            taskScheduler = new TaskScheduler(this);
        }
        #endregion
    }
}
