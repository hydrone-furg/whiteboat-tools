# whiteboat-tools

## Dependencies
```
pip install pyserial
```

## Cloning...
### [Creating a Token](https://github.com/settings/tokens)
```
git clone -b develop https://<your_user>:<your_token>@github.com/hydrone-furg/whiteboat-tools
```

## To use SITL:
### [Rover SITL](https://ardupilot.org/dev/docs/rover-sitlmavproxy-tutorial.html)
#### Generate new token > Generate new token (classic)
```
cd ~/ardupilot/Rover
sim_vehicle.py --map --console
```
