import requests
import math

from turtlebot_control.rotate import  PI

def distance(pos1, pos2):
    return math.sqrt(math.pow((pos1[0]-pos2[0]), 2) + math.pow((pos1[1]-pos2[1]), 2))

def upload(data):
    # sending post request and saving response as response object
    try:
        #r = requests.post(url = config.upload_endpoint, data = data)
        print('upload robot status: {}'.format(data))
    except requests.exceptions.HTTPError as errh:
        print ("Http Error:",errh, "\n on devices: \n", data)
    except requests.exceptions.ConnectionError as errc:
        print ("Error Connecting:",errc, "\n on devices: \n", data)
    except requests.exceptions.Timeout as errt:
        print ("Timeout Error:",errt, "\n on devices: \n", data)
    except requests.exceptions.RequestException as err:
        print ("OOps: Something Else",err, "\n on devices: \n", data)

def radiou2dgree(radius):
    return radius*180/PI