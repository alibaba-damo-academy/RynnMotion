# How to Run Python Scripts

## Setup

```bash
cd ~/Documents/Damowork/RynnMotion/python
source venv/bin/activate
```

## Motion Simulation

### SO101 Robot
```bash
python scripts/motion_sim_robot.py --config config/config_so101.yaml
```

### FR3 Robot
```bash
python scripts/motion_sim_robot.py --config config/config_fr3.yaml
```

### With Mock Communicator (no LCM required)
```bash
python scripts/motion_sim_robot.py --config config/config_so101_mockcom.yaml
python scripts/motion_sim_robot.py --config config/config_fr3_mockcom.yaml
```

## Data Monitor (separate terminal)

### SO101 Robot
```bash
python scripts/robot_state_monitor.py --config config/config_so101.yaml
```

### FR3 Robot
```bash
python scripts/robot_state_monitor.py --config config/config_fr3.yaml
```

Press `q` to quit the monitor.

## Run Both Together

**Terminal 1 - Simulation:**
```bash
cd ~/Documents/Damowork/RynnMotion/python
source venv/bin/activate
python scripts/motion_sim_robot.py --config config/config_fr3.yaml
```

**Terminal 2 - Data Monitor:**
```bash
cd ~/Documents/Damowork/RynnMotion/python
source venv/bin/activate
python scripts/robot_state_monitor.py --config config/config_fr3.yaml
```
