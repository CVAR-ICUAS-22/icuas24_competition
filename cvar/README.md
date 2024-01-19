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


## EXTRA: USE DOCKER COMPOSE (ONLY FOR DEVELOPMENT)

With this you can run the docker for development 

0. Enable access to X server
```
xhost +
```

1. Start container
```
docker compose up -d
```

2. Run as many terms as you need (you can run this in other terminal, from any path)
```
docker exec -it icuas24_focal_nogpu /bin/bash 
```

3. Stop container
```
docker compose down
```





