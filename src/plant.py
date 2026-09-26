import json
import time
import os
from urllib.request import Request, urlopen
URL='http://127.0.0.1:'+os.environ.get('PLANT_PORT','18765')
def state(view=''):
    with urlopen(URL+('/'+view if view else ''),timeout=3) as f:return json.load(f)
def submit(data):
    with urlopen(Request(URL,data=json.dumps(data).encode(),headers={'Content-Type':'application/json'}),timeout=3) as f:return json.load(f)['id']
def wait(token,timeout=120):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        s=state('control')
        if token in s['results']:return s['results'][token]
        time.sleep(.05)
    submit({'op':'stop'})
    raise TimeoutError('plant command timed out')
def command(data,timeout=120):return wait(submit(data),timeout)
def settle(seconds):
    start=state()['t']
    while state()['t']-start<seconds:time.sleep(.05)
