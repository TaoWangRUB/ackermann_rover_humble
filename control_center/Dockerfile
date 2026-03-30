# Stage 1: Build React UI
FROM node:20-slim AS ui-builder
WORKDIR /app/ui
COPY ui/package.json ui/package-lock.json* ./
RUN npm install
COPY ui/ ./
RUN npm run build

# Stage 2: Python runtime
FROM python:3.11-slim
WORKDIR /app

# Install protoc for proto generation
RUN apt-get update && apt-get install -y --no-install-recommends \
    protobuf-compiler && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY main.py .
COPY cc/ cc/
COPY proto/ proto/
COPY config/ config/

# Generate Python protobuf stubs
RUN protoc --python_out=proto/ --proto_path=proto/ proto/rover_health.proto

# Copy built UI
COPY --from=ui-builder /app/ui/dist /app/static

EXPOSE 8080

CMD ["python", "main.py"]
