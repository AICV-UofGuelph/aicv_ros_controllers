# sp_controller
This ROS package contains the PI trajectory tracking controller used by the RB-Kairos robot. The PI controller itself is ```sp_pid.py```.

Running the controller relies on a list of seperated coordinate points to follow. To generate these points at the correct distance apart so that the robot runs at the desired speed, ```pipeline_sp_pid.py``` is used.
## pipeline_sp_pid.py

### Compile/Run
```
$ python pipeline_sp_pid.py csv_file [-t timestep] [-f folder_name]
```

### Running with pipeline_create_waypoints.py
```
$ python pipeline_sp_pid.py `python pipeline_create_waypoints.py file_folder [-t timestep] [-f folder_name]`
```
- where `file_folder` is name of file that contains a path file, map file, and yaml file
- where `folder_name` is the name of the directory the run data will be saved in 

### Notes

- if not given, `timestep` variable will be set to 0.2
- if not given, `folder_name` variable will be set to run_[num of files in 'run_data/' directory]
- use Python 2.7 to run
- make amcl values in config file smaller before using
