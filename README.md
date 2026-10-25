# ftc-biobuzz

robot code for biobuzz. it's command based, using ivy for commands and pedro pathing for driving.

you need jdk 17.

## make commands

```sh
make help         # list all the commands
make build        # build the apk
make test         # run the tests
make deploy       # put the code on the robot (use this one normally)
make install      # full apk install
make hot          # quick hot reload with sloth, teamcode only
make connect      # wait for the control hub to connect
make scrcpy       # show the control hub's screen
make pull-logs    # download the robot logs to robot-logs/
make pull-logs 2  # download just the newest 2 logs
make viewer       # open maxscope to look at logs (http://127.0.0.1:8008)
```

`make deploy` figures out if it needs a full install or can just hot reload.

## where stuff is

- [teamcode](TeamCode/src/main/): opmodes, subsystems, vision, and pedro config
- [core](TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/core/): shared framework code

## docs

- [development](dutchdocs/DEVELOPMENT.md): subsystems, controls, autos
- [operations](dutchdocs/OPERATIONS.md): hardware setup and tuning
- [logging](dutchdocs/LOGGING.md): logs and post-match debugging
- [progress](dutchdocs/PROGRESS.md): lab notes
- [ai guide](AI-GUIDE.md): rules for coding assistants
