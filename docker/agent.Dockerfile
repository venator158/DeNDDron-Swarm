FROM python:3.10-slim

WORKDIR /app

# Install dependencies first for better caching
COPY src/agent/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Agent code plus the modules shared with the ship
COPY src/agent/ ./
COPY src/common/ ./

CMD ["python", "-u", "main.py"]
