FROM python:3.8-slim-buster

WORKDIR /app

COPY requirements.txt requirements.txt
RUN apt-get update
RUN apt-get install -y build-essential
RUN pip3 install -r requirements.txt

COPY . .

CMD [ "python3", "log_to_influx.py"]
