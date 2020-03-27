import copy
import datetime
from influxdb import InfluxDBClient

import config

body_pos = {
    'measurement': config.Table_Name_Robot_Pos,
    'time': 0,
    'tags': {
        'robot_id': 0,
        'inspection_id': 0,
    },
    'fields':{
        'pos_x': 0,
        'pos_y': 0,
        'pos_angle': 0
    }
}

body_event = {
    'measurement': config.Table_Name_Robot_Event,
    'time': 0,
    'tags': {
        'robot_id': 0,
        'inspection_id': 0,
        'waypoint_no': 0
    },
    'fields':{
        'enter_time': 0,
        'leave_time': 0
    }
}

class DBHelper():
    def __init__(self, db_url=config.upload_URL, port=config.upload_PORT, dbname=config.upload_DB):
        self.client = InfluxDBClient(host=db_url, port=port, database=dbname)

    def writePosRecord(self, inspection_id, robot_id, pos_records):
        records = []
        if len(pos_records) == 0:
            return

        for record in pos_records:
            body = copy.deepcopy(body_pos)
            x, y, angle, timestamp = record[0], record[1], record[2], record[3]

            body['time'] = timestamp
            body['tags']['robot_id'] = robot_id
            body['tags']['inspection_id'] = inspection_id
            body['fields']['pos_x'] = x
            body['fields']['pos_y'] = y
            body['fields']['pos_angle'] = angle

            records.append(body)

        try:
            self.client.write_points(records)
        except Exception as e:
            print('DB operation: write robot position record error!', e)
    
    def emptyPos(self):
        self.client.query("delete from {};".format(config.Table_Name_Robot_Pos))
        # self.client.query("drop measurement {}".format(config.Table_Name_Robot_Pos))

    def getAllPos(self):
        resutls = self.client.query('select * from {};'.format(config.Table_Name_Robot_Pos))
        return resutls

    def writeEventRecord(self, inspection_id, robot_id, event_records):
        records = []
        if len(event_records) == 0:
            return

        for record in event_records:
            body = copy.deepcopy(body_event)
            waypoint_no, enter_time, leave_time = record[0], record[1], record[2]
            body['time'] = leave_time
            body['tags']['robot_id'] = robot_id
            body['tags']['inspection_id'] = inspection_id
            body['tags']['waypoint_no'] = waypoint_no
            body['fields']['enter_time'] = enter_time
            body['fields']['leave_time'] = leave_time
            records.append(body)
        
        try:
            self.client.write_points(records)
        except:
            print('DB operation: write robot position record error!')

    def emptyEvents(self):
        self.client.query("delete from {};".format(config.Table_Name_Robot_Event))

    def getAllEvents(self):
        resutls = self.client.query('select * from {};'.format(config.Table_Name_Robot_Event))
        return resutls

    def upload(self, inspection_id, robot_id, pos_records, event_records):
        self.writePosRecord(inspection_id, robot_id, pos_records)
        print('DBHelper: sent {} pos records'.format(len(pos_records)))
        self.writeEventRecord(inspection_id, robot_id, event_records)
        print('DBHelper: sent {} event records'.format(len(event_records)))


if __name__ == '__main__':
    dbtool = DBTool()

    cur_time = datetime.datetime.utcnow().isoformat("T")

    dbtool.emptyPos()
    records = [(0,0,0,cur_time)]
    dbtool.writePosRecord(0, 0, records)
    results = dbtool.getAllPos()
    print(results)

    dbtool.emptyEvents()
    records = [(0,cur_time,cur_time)]
    dbtool.writeEventRecord(0, 0, records)
    results = dbtool.getAllEvents()
    print(results)

