# sp_controller
controllers for rbkairos robots

## pipeline_sp_pid.py

### Compile/Run
```
$ python pipeline_sp_pid.py csv_file [time_step]
```

### Running with pipeline_create_waypoints.py
```
$ python pipeline_sp_pid.py `python pipeline_create_waypoints.py file_folder [time_step]`
```
(where `file_folder` is name of file that contains a path file, map file, and yaml file)

### Notes

- if not given, `time_step` variable will be set to 0.2
