package org.firstinspires.ftc.teamcode.feature;

import androidx.annotation.NonNull;

import java.util.PriorityQueue;

class Task implements Comparable<Task> {
    private final Runnable task;
    private final long delay;

    public Task(Runnable task, long delay) {
        this.task = task;
        this.delay = delay;
    }

    public void run() {
        task.run();
    }

    public boolean isReady(long currentTime) {
        return currentTime >= delay;
    }

    @Override
    public int compareTo(Task task) {

        if (this.delay > task.delay)
            return 1;
        else if (this.delay < task.delay)
            return -1;
        return 0;
    }
}

public class Schedule {
    private Schedule() {}
    private static final PriorityQueue<Task> tasks = new PriorityQueue<>();

    public static void addTask(@NonNull Runnable task, double delay) {
        tasks.add(new Task(task, System.nanoTime() + (long)(delay * 1e9)));
    }

    public static void update() {
        long currentTime = System.nanoTime();
        while (!tasks.isEmpty()) {
            if (!tasks.peek().isReady(currentTime)) break;
            tasks.poll().run();
        }
    }

    public static void stop() {
        tasks.clear();
    }
}