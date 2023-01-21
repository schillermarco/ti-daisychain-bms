import cantools
import can
from influxdb_client import InfluxDBClient, Point, WriteOptions

def main():
    database = "bms_1"

    client = InfluxDBClient(url='http://localhost:8086', org='-')

    write_api = client.write_api()

    db = cantools.database.load_file('bms.dbc')
    can_bus = can.interface.Bus('can0', bustype='socketcan')
    while(True):
        try:
            message = can_bus.recv()
            decoded_message = db.decode_message(message.arbitration_id, message.data)

            points = []
            for key in decoded_message:
                point = Point(key).field("value", decoded_message[key])
                points.append(point)

            write_api.write(bucket=database, record=points)
        except:
            pass

if __name__ == "__main__":
    main()
