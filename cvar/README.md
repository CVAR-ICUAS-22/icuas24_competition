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

Pull all repositories:
```
vcs pull -n
```

## Build docker
Build docker:
```
./docker_build.sh --focal-nogpu
```

## Run docker
Go to cvar workspace:
```
cd cvar
```

Run docker:
```
docker compose up -d
```

## Exec docker
For first time, compile the workspace. Inside docker:
```
cd ~/cvar_ws
catkin build
```

Exec docker:
```
docker exec -it icuas24_competition /bin/bash
```

## Stop docker
Stop docker:
```
docker compose down
```


## EXTRA (ONLY FOR DEVELOPMENT)
With this you can run the docker for development 

0. Enable access to X server
```
xhost +
```
