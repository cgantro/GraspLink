# Diagnostics

`grasplink_diagnostics` is a dependency-free C++17 library for asynchronous logs, metrics, and execution timings. `Logger` owns the only file writer thread; producers copy records into a bounded queue and never write to the file. The default output is `logs/grasplink.jsonl` relative to the process working directory. Each JSON Lines record includes a Unix timestamp in milliseconds and the producer thread identifier.

`Profiler::Trace` decorates a callable at a composition boundary. It records elapsed nanoseconds, logs standard and unknown exceptions, then rethrows them. `Profiler::Measure` supports a named RAII scope, and `Profiler::Metric` records a numeric value with an explicit unit. These APIs let the application attach diagnostics without placing file or logging code in controller, kinematics, or physics logic.

```cpp
grasplink::diagnostics::Logger logger;
grasplink::diagnostics::Profiler profiler(logger);

profiler.Trace("simulation.step", [&] {
    physics.Step(deltaSeconds);
});
profiler.Metric("simulation.delta", deltaSeconds, "s");
```

The queue defaults to 8192 records. When it is full, the new record is discarded so a producer such as the fixed control loop does not wait for disk I/O. Error records can replace a queued metric, profile, debug, or info record. `GetDroppedRecordCount()` exposes cumulative loss, and `GetWriteFailureCount()` reports file open, write, and flush failures. `Flush()` waits for accepted records; `Shutdown()` stops accepting records, drains the queue, and joins the worker. The application keeps the logger alive through subsystem teardown and shuts it down last.

The worker receives only copied strings and numbers. It never accesses Flecs, OpenGL, Jolt, or controller objects. File-open and write failures are reported directly to standard error to avoid recursively sending a sink failure through the same logger.

Each record is serialized by the worker. String fields escape JSON quotes, backslashes, and control bytes directly into the output stream, preserving UTF-8 bytes and avoiding a temporary escaped string for every field. Numeric formatting uses the classic locale so the decimal separator remains valid JSON regardless of the process locale.
