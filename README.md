# Robaction

The robot action programming for lecture.

## Manual Installation and Usage

Cloning and getting source codes from Roboken repositories to deploy the YP-Spur environment.

```
make
```

To run with ypspur-coordinator:

```
bin/coordinator speego /dev/ttyACM1
```

## Docker Installation and Usage

Build the docker container that operates the robot.

```
docker build -t ypspur .
```

After that, you can ready to run the robot to specify these parametes like as:

```
docker run -it -d \
  --name ypspur_coordinator \
  --device=/dev/ttyACM0 \
  --device=/dev/ttyACM1 \
  -e PARAM=speego \
  -e SENSOR=/dev/ttyACM0 \
  -e MICON=/dev/ttyACM1 \
  -e USE_ODOMETRY=0 \
  ypspur
```

To debug on docker:

```
# view coordinator logs
docker logs ypspur_coordinator
# or enter the operation environment
docker exec -it ypspur_coordinator bash
```

## Actions

This repository has these actions which a self-propelled robot runs for.

 * `test`: Checking whether it runs currectly.
 * `eight`: Running eight-figure.
 * `dist`: Running if the front destination is more than 1m.
 * `round`: Running around a pillar.
 * `hug`: The "Hansel and Gretel."

To try the above actions, change directory to specific action dir, build and execute it.

## Coordinator Support

The Robaction support coordinator (YP-Spur).

Before using actions, start up the coordinator to type `bin/coordinator`. Do not forget connect and switch on the machine.

## Odometry Logging Support

The Robaction supports odometry logging.

You can use this feature to type `bin/odometry` from root directory.

## License

MIT License
