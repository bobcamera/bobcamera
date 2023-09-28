FROM python:3-alpine

## Pip dependencies
# Upgrade pip
RUN pip install --upgrade pip
# Install production dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt