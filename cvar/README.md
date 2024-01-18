# FOR CVAR DEVELOPERS 

## Import repository
Go to cvar workspace:
```
cd cvar
```

Clone using vcstool:
```
vcs import < repositories.repos
```

## Build docker

Build docker:
```
./docker_build.sh --focal-nogpu
```

## Run docker

Run docker:
```
./docker_run.sh --focal-nogpu --dev
```

For first time, compile the workspace. Inside docker:
```
cd ~/cvar_ws
catkin build
```

## Exec docker

Exec docker:
```
docker exec -it icuas24_competition /bin/bash
```

## Stop docker

Stop docker:
```
docker container rm -f icuas24_competition
```