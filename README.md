# deepracer-2023

## Steps to create deepracer spot instance
- git clone https://github.com/aws-deepracer-community/deepracer-on-the-spot
- cd deepracer-on-the-spot
  ```
  ./create-base-resources.sh racingwarriors-ravi-base-v2 [IP_ADDRESS]
  ```
-  cd custom-files
-  ./create-spot-instance.sh base firstmodelspot 30
   Current : ./create-spot-instance.sh racingwarriors-ravi-base racingwarriors-ravi-model-spot 30
```
   ./create-standard-instance.sh racingwarriors-ravi-base-v2 RacingWarriors-Ravi-Model-V1 60
```

## Steps to shutdown deepracer spot instance
- sudo shutdown now

## Deepracer log guru
D:\machine learning\Deepracer\deepracer-log-guru>python.exe -m src.main.guru
https://github.com/aws-deepracer-community/deepracer-log-guru/blob/master/docs/installation.md
