# Test Report
Generated: 2026-02-17 21:38:45Z

Quality Gate thresholds: legal/copyright=0,whitespace/comments=0,whitespace/line_length=0,whitespace/tab=0
Ignored categories: <none>

## cpplint (ament_cpplint)
```
Using '--root=/home/taowang/workspace/ackermann_rover_humble/src/ackermann_control/src' argument

Done processing /home/taowang/workspace/ackermann_rover_humble/src/ackermann_control/src/ackermann_controller.cpp

Using '--root=/home/taowang/workspace/ackermann_rover_humble/src/safety/src' argument

Done processing /home/taowang/workspace/ackermann_rover_humble/src/safety/src/watchdog_node.cpp

No problems found
```

## cppcheck
```
Checking src/ackermann_control/src/ackermann_controller.cpp ...
1/2 files checked 60% done
Checking src/safety/src/watchdog_node.cpp ...
2/2 files checked 100% done
```
## Quality Gate

- No cpplint categories reported.

## Build & Test
Starting >>> ackermann_control
Starting >>> safety
Finished <<< ackermann_control [7.22s]
Finished <<< safety [7.23s]

Summary: 2 packages finished [7.34s]
Starting >>> ackermann_control
Starting >>> safety
Finished <<< safety [0.17s]
Finished <<< ackermann_control [0.19s]

Summary: 2 packages finished [0.35s]
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
