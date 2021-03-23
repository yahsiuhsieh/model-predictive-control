# Model Predictive Control of an Autonomous Vehicle

We utilize model predictive controller to perform lane following and obstacle avoidance.

<p align="center">
  <img width="450" height="300" src="https://github.com/coldhenry/Model-Predictive-Control-of-Autonomous-Car/blob/main/results/obstacle_avoidance.gif"/><br/>
  <em>Agent trying to avoid obstacles.</em>
</p>

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Simulation Environment

[Multi-Purpose MPC by matssteinweg](https://github.com/matssteinweg/Multi-Purpose-MPC)

### Built With

* Python 3.6.10

* do-mpc 4.1.1

* numpy >= 1.16.2

* matplotlib >= 3.1.1

## Code Organization

```
.
├── src                    
│   ├── main.py            # Execution part
│   ├── MPC.py             # the algorithm of model predictive control
│   ├── model.py           # simple bicycle model
│   ├── globals.py         # some variables that use globally
│   ├── maps.py            # generate a usable map from any picture (cited from matssteinweg)
│   └── reference_path.py  # generate reference path, waypoints for the assigned map (cited from matssteinweg)
├── result                 # GIF files of the results of two scenarios
├── maps                   # the picture of the map
└── README.md
```

## How to Run

There are 2 tasks you can try, which are lane following and obstacle avoidance, as shown in [Results section](#results)

Modify the flag at line 95 in the `main.py` file if you want to switch between tasks

```python
use_obstacles = False
```

after that, just run

```python
python main.py
```

## Results

<p align="center">
  <img width="450" height="300" src="https://github.com/coldhenry/Model-Predictive-Control-of-Autonomous-Car/blob/main/results/lane_following.gif"/><br/>
</p>
<p align="center">
  <em>Agent trying to follow desired trajectory.</em>
</p>


## Authors

* **Arthur Hsieh** - <i>Initial Works</i> - [arthur960304](https://github.com/arthur960304)
* **Henry Liu** - <i>Initial Works</i> - [coldhenry](https://github.com/coldhenry)
