from flask import Flask

hhlogcheck1 = 0
hhlogcheck2 = 0
hhlogcheck3 = 0
irlogcheck1 = 0
irlogcheck2 = 0
timecheck1 = 0
timecheck2 = 0

app = Flask(__name__)


@app.route('/helloesp')
def helloHandler():
    return 'Hello ESP8266, from Flask'


@app.route('/HHchair')
def chair():

    return 'HH signal received'


@app.route('/HHsink')
def sink():
    return 'HH signal received'


@app.route('/HHtable')
def table():
    return 'HH signal received'


@app.route('/HHtable2')
def table2():
    return 'HH signal received'


app.run(host='0.0.0.0', port=8090)