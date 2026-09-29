FROM python:3.10-slim

WORKDIR /app

RUN pip install --no-cache-dir eclipse-zenoh==1.10.1

# Ship C2 + dashboard plus the modules shared with the agents
COPY src/ship/ ./
COPY src/common/ ./

EXPOSE 8080
CMD ["python", "-u", "ship.py"]
