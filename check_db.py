from influxdb import InfluxDBClient

from utils.tsdb import DBHelper
import datetime

client = InfluxDBClient('www.bestfly.ml', 8086, database='robot')

result = client.query('select * from robot_poss')
print('pos records {} \n'.format(len(result)))
print(result)
print('\n drop measurement robot_poss')
result = client.query('drop measurement robot_poss')

result = client.query('select * from robot_event')
print('pos records {} \n'.format(len(result)))
print(result)
print('\n drop measurement robot_event')
result = client.query('drop measurement robot_event')