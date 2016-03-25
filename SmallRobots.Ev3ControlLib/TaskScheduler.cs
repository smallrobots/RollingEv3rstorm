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

using MonoBrickFirmware.Display;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace SmallRobots.Ev3ControlLib
{
    /// <summary>
    /// Task timing structure associated to each scheduled task
    /// by the TaskScheduler task
    /// </summary>
    public partial class TaskTimingStructure : IComparable
    {
        #region Fields
        /// <summary>
        /// Number of execution of the associated Task so far
        /// </summary>
        public long numberOfExecutions;

        /// <summary>
        /// Next due time for the associated task
        /// </summary>
        public long nextDueTime;
        #endregion

        #region Constructors
        /// <summary>
        /// Constructs a new timing structure
        /// </summary>
        /// <param name="theNextDueTime">Next due time for the associated Task</param>
        public TaskTimingStructure(long theNextDueTime)
        {
            // Fields initialization
            nextDueTime = theNextDueTime;
            numberOfExecutions = 0;
        }

        /// <summary>
        /// Implementazione interfaccia IComparable
        /// </summary>
        public int CompareTo(object obj)
        {
            return nextDueTime.CompareTo(((TaskTimingStructure)obj).nextDueTime);
        }
        #endregion
    }

    /// <summary>
    /// High precision task scheduler class
    /// </summary>
    public partial class TaskScheduler
    {
        #region Fields
        /// <summary>
        /// Constant delay to be applied to the task is scheduled at a time
        /// where a previous task has already been scheduled
        /// </summary>
        protected int standardDelayWhenOverscheduled;

        /// <summary>
        /// True if the execution log has been requested on Start()
        /// </summary>
        protected bool withLog;

        /// <summary>
        /// The current queue of scheduled tasks
        /// </summary>
        // protected SortedList<long, PeriodicTask> taskQueue;
        protected SortedList<TaskTimingStructure, PeriodicTask> taskQueue;

        /// <summary>
        /// The execution log
        /// </summary>
        protected string executionLog;

        /// <summary>
        /// The high precision time counter used by this scheduler
        /// </summary>
        protected Stopwatch stopWatch;

        /// <summary>
        /// The scheduler thread
        /// </summary>
        protected Thread schedulerThread;

        /// <summary>
        /// Next due time expressed in milliseconds (ms)
        /// </summary>
        protected int dueTime;

        /// <summary>
        /// True when the scheduler must work, false to stop it
        /// </summary>
        protected bool keepScheduling;

        /// <summary>
        /// The robot for which this scheduler has been built
        /// </summary>
        protected Robot robot;
        #endregion

        #region Properties
        /// <summary>
        /// Gets the current queue of scheduled tasks
        /// </summary>
        // public SortedList<long, PeriodicTask> TaskQueue
        public SortedList<TaskTimingStructure, PeriodicTask> TaskQueue
        {
            get
            {
                return taskQueue;
            }
        }

        /// <summary>
        /// Gets the executionlog
        /// </summary>
        public string LastExecutionLog
        {
            get
            {
                return executionLog;
            }
        }

        /// <summary>
        /// Elapsed time since the scheduler has been started
        /// expressd in milliseconds (ms)
        /// </summary>
        public long ElapsedTime
        {
            get
            {
                return stopWatch.ElapsedMilliseconds;
            }
        }
        #endregion

        #region Constructors
        /// <summary>
        /// Constructs a new Task Scheduler
        /// </summary>
        public TaskScheduler()
        {
            // Fields initialization
            init();
        }

        /// <summary>
        /// Constructs a new Task Scheduler for a particular robot
        /// </summary>
        /// <param name="theRobot">The robot for which this scheduler must be built</param>
        public TaskScheduler(Robot theRobot)
        {
            // Fields initialization
            init();
            robot = theRobot;
        }

        /// <summary>
        /// Fields initialization
        /// </summary>
        void init()
        {
            executionLog = "";
            dueTime = 0;
            // taskQueue = new SortedList<long, PeriodicTask>();
            taskQueue = new SortedList<TaskTimingStructure, PeriodicTask>();
            stopWatch = new Stopwatch();
            schedulerThread = new Thread(theScheduler);
            standardDelayWhenOverscheduled = 1;
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Add a new Periodic Task to the Scheduler
        /// </summary>
        /// <param name="thePeriodicTask">The Periodic Task to Add</param>
        public void Add(PeriodicTask thePeriodicTask)
        {
            //taskQueue.Add(thePeriodicTask.Period, thePeriodicTask);

            // Checks if another task has been already scheduled at the same time
            // long nextDueTime = thePeriodicTask.Period;
            long nextDueTime = 0;
            bool scheduled = false;
            while (!scheduled)
            {
                if (AlreadyScheduled(nextDueTime))
                {
                    nextDueTime = nextDueTime + standardDelayWhenOverscheduled;
                }
                else
                {
                    scheduled = true;
                }
            }
            taskQueue.Add(new TaskTimingStructure(nextDueTime), thePeriodicTask);
        }

        /// <summary>
        /// Gets the next task to be executed
        /// </summary>
        /// <returns></returns>
        public PeriodicTask GetNextTask()
        {
            if ((taskQueue == null) || (taskQueue.Count == 0))
            {
                // taskQueue null or empty
                return null;
            }
            else
            {
                //return (PeriodicTask) taskQueue.GetByIndex(0);
                return taskQueue.Values[0];
            }
        }

        /// <summary>
        /// Start the task scheduler
        /// </summary>
        /// <param name="withLog">Set to true to keep an execution log</param>
        /// <param name="asyncExecution">Set to true if the the scheduler will be stopped 
        ///                              from within</param>
        public void Start(bool withLog = false, bool asyncExecution = false)
        {
            // Keep the execution log
            this.withLog = withLog;

            // Start the scheduler thread
            keepScheduling = true;
            schedulerThread.Start();

            if (!asyncExecution)
            {
                // Wait for scheduler thread termination
                schedulerThread.Join();
            }
        }

        /// <summary>
        /// Stops the scheduler
        /// </summary>
        public void Stop()
        {
            // Stop the execution log
            if (withLog)
            {
                executionLog = executionLog + "Execution stopped by user.";
            }

            // Stops the scheduler thread
            keepScheduling = false;
            // schedulerThread.Join();

            withLog = false;
        }
        #endregion

        #region Protected methods
        /// <summary>
        /// The scheduler algorithm
        /// </summary>
        protected void theScheduler()
        {
            // Sleeps until first due time
            stopWatch.Reset();
            stopWatch.Start();
            // LcdConsole.WriteLine("Stopwatch started");

            if (taskQueue.Count != 0)
            {

                while (keepScheduling)
                {
                    // Gets current time task data
                    long msec = stopWatch.ElapsedMilliseconds;
                    long dueTime = taskQueue.Keys[0].nextDueTime;
                    long numberOfExecutions = taskQueue.Keys[0].numberOfExecutions;
                    // LcdConsole.WriteLine("Extracting data");

                    // Time to wait until next execution
                    int rest = (int) (dueTime - msec);

                    // Put this thread to sleep if the dueTime is further than 200ms
                    if (rest > 200)
                    {
                        // LcdConsole.WriteLine("Waiting asleep");
                        Thread.Sleep(rest - 200);
                    }

                    // When dueTime is closer than 200ms, this function actively waits the stopwatch
                    while (true)
                    {
                        // LcdConsole.WriteLine("Waiting awake");
                        if (stopWatch.ElapsedMilliseconds >= msec + rest)
                        {
                            break;
                        }
                    }

                    // When the stopwatch reaches the dueTime, the callBack function is invoked
                    // But before checks the the scheduler has not been stopped
                    // while waiting
                    if (keepScheduling)
                    {
                        PeriodicTask task = TaskQueue.Values[0];
                        if (withLog)
                        {
                            // Logs the execution time
                            executionLog = executionLog +
                                            stopWatch.ElapsedMilliseconds.ToString() + " " +
                                            task.Name + "\r\n";
                        }

                        // Executes the action
                        // LcdConsole.WriteLine("Launching action " + task.Name);
                        LcdConsole.WriteLine("Sched error: " + Math.Abs(dueTime - stopWatch.ElapsedMilliseconds).ToString("F2"));
                        task.Action(robot);

                        // Determines next due time
                        // LcdConsole.WriteLine("Computes next due time");
                        long nextDueTime = (numberOfExecutions + 1) * task.Period;
                        bool slotFound = false;

                        // Updates the due time
                        PeriodicTask updatedTask = task;
                        taskQueue.RemoveAt(0);

                        while (!slotFound)
                        {
                            // Check for task already scheduled at the same time
                            if (AlreadyScheduled(nextDueTime))
                            {
                                // If a task is already scheduled at the same time
                                // try next millisecond
                                nextDueTime = nextDueTime + standardDelayWhenOverscheduled;
                            }
                            else
                            {
                                // First task to be scheduled at this nextDueTime slot
                                slotFound = true;
                            }
                        }

                        // schedules the task
                        TaskTimingStructure key = new TaskTimingStructure(nextDueTime);
                        key.numberOfExecutions = numberOfExecutions + 1;
                        taskQueue.Add(key, task);                        
                    }
                }
            }
            stopWatch.Stop();
        }

        /// <summary>
        /// Checks if there already is a task scheduled for
        /// the supplied nextDueTime
        /// </summary>
        /// <param name="nextDueTime">Due time to be tested</param>
        /// <returns></returns>
        protected bool AlreadyScheduled(long nextDueTime)
        {
            bool alreadyScheduled = false;

            for (int i = 0; i < taskQueue.Count; i++)
            {
                if (taskQueue.Keys[i].nextDueTime == nextDueTime)
                {
                    alreadyScheduled = true;
                    break;
                }
            }

            return alreadyScheduled;
        }
        #endregion
    }
}
