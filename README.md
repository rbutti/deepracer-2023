# deepracer-2023

## Steps to create deepracer spot instance
- git clone https://github.com/aws-deepracer-community/deepracer-on-the-spot
- cd deepracer-on-the-spot
-  ./create-base-resources.sh base [IP_ADDRESS]
-  cd custom-files
-  ./create-spot-instance.sh base firstmodelspot 30
   Current : ./create-spot-instance.sh racingwarriors-ravi-base racingwarriors-ravi-model-spot 30

## Steps to shutdown deepracer spot instance
- sudo shutdown now
