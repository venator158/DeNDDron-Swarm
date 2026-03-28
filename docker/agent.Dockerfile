FROM python:3.10-slim

WORKDIR /app

# Install dependencies first for better caching
COPY src/agent/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the python agent code
COPY src/agent/ ./

# Run the agent directly
CMD ["python", "-u", "main.py"]
