# Robaction

The robot action programming for lecture.

## Installation

Cloning and getting source codes from Roboken repositories to deploy the YP-Spur environment.

```
make
```

## Actions

This repository has these actions which a self-propelled robot runs for.

 * `test`: Checking whether it runs currectly.
 * `eight`: Running eight-figure.
 * `dist`: Running if the front destination is more than 1m.

To try the above actions, change directory to specific action dir, build and execute it.

## Coordinator Support

The Robaction support coordinator (YP-Spur).

Before using actions, start up the coordinator to type `bin/coordinator`. Do not forget connect and switch on the machine.

## Odometry Logging Support

The Robaction supports odometry logging.

You can use this feature to type `bin/odometry` from root directory.

## License

MIT License
