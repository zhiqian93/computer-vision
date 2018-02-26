import urllib3

http = urllib3.PoolManager()
r = http.request('GET', 'http://192.168.4.1:80/event')