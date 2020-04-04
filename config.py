RobotID = 'tb3_0'
Holding_Step_Time = 20
Holding_Time_Variance = 1
Circle_Rotate_Steps = 4
Rotate_Speed = 30
Valid_Range_Radius = 0.1

Holding_Time = Holding_Step_Time+360/Rotate_Speed+Holding_Time_Variance

#tsdb
upload_URL = 'www.bestfly.ml'
upload_PORT = 8086
upload_DB = 'robot'
Table_Name_Robot_Pos = 'robot_poss'
Table_Name_Robot_Event = 'robot_event'

Pos_Collect_Interval = 0.2
Upload_Interval = 2
