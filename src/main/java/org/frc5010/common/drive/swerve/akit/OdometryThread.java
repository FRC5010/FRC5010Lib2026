// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Abstract base class for odometry threads, providing common functionality for managing signals,
 * queues, and periodic updates.
 */
public abstract class OdometryThread extends Thread {
  /** Shared lock that synchronises the odometry thread with the main robot periodic loop. */
  public static final ReentrantLock odometryLock = new ReentrantLock();

  protected final Lock signalsLock = new ReentrantLock();
  protected final List<DoubleSupplier> genericSignals = new ArrayList<>();
  protected final List<Queue<Double>> genericQueues = new ArrayList<>();
  protected final List<Queue<Double>> timestampQueues = new ArrayList<>();
  protected static OdometryThread commonInstance;

  public static OdometryThread getInstance() {
    return commonInstance;
  }
  /** Creates a new odometry thread with the specified name. */
  protected OdometryThread(String threadName) {
    setName(threadName);
    setDaemon(true);
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty() && RobotBase.isReal()) {
      super.start();
    }
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    OdometryThread.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      OdometryThread.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    OdometryThread.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      OdometryThread.odometryLock.unlock();
    }
    return queue;
  }

  /** Abstract method to be implemented by subclasses for specific signal handling. */
  protected abstract void runThreadLogic();

  @Override
  public void run() {
    while (true) {
      runThreadLogic();
    }
  }
}
