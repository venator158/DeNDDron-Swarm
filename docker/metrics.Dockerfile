FROM python:3.10-slim

WORKDIR /app

# Install dependencies (only zenoh is needed for basic metrics)
RUN pip install --no-cache-dir eclipse-zenoh==1.10.1

COPY src/metrics/ ./src/metrics/

CMD ["python", "src/metrics/main.py"]
